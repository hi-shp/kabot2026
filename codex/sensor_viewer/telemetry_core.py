"""Bounded sensor conversions. Display units are degrees, metres and SI rates."""
from collections import deque
import math
import struct
import time

import numpy as np

POINT_HEADER = struct.Struct('<4sIIdI')
FIELD_DTYPES = {1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}


def finite(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def vector(value, scale=1.):
    return {key: finite(getattr(value, key) * scale) for key in ('x','y','z')}


def degrees360(value):
    value = finite(value)
    return value % 360 if value is not None else None


def quaternion_angles(q):
    values = [finite(getattr(q, key)) for key in ('x','y','z','w')]
    if None in values:
        return None
    length = math.sqrt(sum(x*x for x in values))
    if length < 1e-9:
        return None
    x,y,z,w = [value/length for value in values]
    signed = {'roll':math.degrees(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))),
              'pitch':math.degrees(math.asin(max(-1.,min(1.,2*(w*y-z*x))))),
              'yaw':math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))}
    return {'deg_360':{k:degrees360(v) for k,v in signed.items()},'deg_signed':signed,
            'quaternion':dict(zip(('x','y','z','w'),(x,y,z,w)))}


def imu_value(message):
    orientation = quaternion_angles(message.orientation)
    if len(message.orientation_covariance) and message.orientation_covariance[0] < 0:
        orientation = None
    return {'frame_id':message.header.frame_id,'orientation':orientation,
            'angular_velocity_deg_s':vector(message.angular_velocity,180/math.pi),
            'acceleration_m_s2':vector(message.linear_acceleration)}


def vectornav_value(message):
    bits = message.group_fields
    result = {'frame_id':message.header.frame_id,'group_fields':bits}
    if bits & 0x0008:
        # VectorNav raw yaw/pitch/roll are already degrees, in this order.
        signed = dict(zip(('yaw','pitch','roll'),(message.yawpitchroll.x,message.yawpitchroll.y,message.yawpitchroll.z)))
        result.update(deg_360={k:degrees360(v) for k,v in signed.items()},
                      deg_signed={k:finite(v) for k,v in signed.items()},reference='VectorNav raw NED')
    if bits & 0x0020:
        result['angular_velocity_deg_s'] = vector(message.angularrate,180/math.pi)
    if bits & 0x0100:
        result['acceleration_m_s2'] = vector(message.accel)
    return result


def scan_value(message, maximum=900):
    values = np.asarray(message.ranges,dtype=np.float32)
    indices = np.arange(len(values))
    valid = np.isfinite(values) & (values > max(0.,message.range_min)) & (values <= message.range_max)
    candidates = indices[valid]
    nearest = None
    if candidates.size:
        index = int(candidates[np.argmin(values[candidates])])
        nearest = {'distance_m':float(values[index]),
                   'azimuth_deg':degrees360(math.degrees(message.angle_min+index*message.angle_increment))}
    if candidates.size > maximum:
        candidates = candidates[np.linspace(0,len(candidates)-1,maximum,dtype=np.int64)]
    points = [[round(degrees360(math.degrees(message.angle_min+int(i)*message.angle_increment)),3),round(float(values[i]),3)] for i in candidates]
    return {'frame_id':message.header.frame_id,'rays':len(values),'valid_rays':int(np.count_nonzero(valid)),
            'range_min_m':finite(message.range_min),'range_max_m':finite(message.range_max),
            'nearest':nearest,'polar_deg_m':points}


def cloud_value(message, maximum=5000, max_range=100.):
    """Structured zero-copy view respects field offsets, endianness and row padding."""
    fields = {f.name:f for f in message.fields}
    for key in ('x','y','z'):
        if key not in fields or fields[key].count != 1 or fields[key].datatype not in FIELD_DTYPES:
            raise ValueError('PointCloud2 requires scalar x/y/z fields')
    if not 1 <= maximum <= 30000 or not 0 < max_range <= 1000:
        raise ValueError('Invalid point budget or range')
    if message.point_step <= 0 or message.width < 0 or message.height < 0:
        raise ValueError('Invalid cloud dimensions')
    if message.row_step < message.width*message.point_step:
        raise ValueError('Invalid PointCloud2 row_step')
    count = message.width*message.height
    if count > 2_000_000:
        raise ValueError('Cloud exceeds point input budget')
    needed = (message.height-1)*message.row_step+message.width*message.point_step if count else 0
    if len(message.data) < needed:
        raise ValueError('Truncated PointCloud2 buffer')
    names = ['x','y','z']
    if 'ring' in fields and fields['ring'].count == 1 and fields['ring'].datatype in FIELD_DTYPES:
        names.append('ring')
    endian = '>' if message.is_bigendian else '<'
    formats = [endian+FIELD_DTYPES[fields[key].datatype] for key in names]
    dtype = np.dtype({'names':names,'formats':formats,'offsets':[fields[k].offset for k in names],'itemsize':message.point_step})
    if count:
        view = np.ndarray((message.height,message.width),dtype=dtype,buffer=memoryview(message.data),strides=(message.row_step,message.point_step))
        xyz = np.column_stack([view[k].reshape(-1) for k in ('x','y','z')]).astype(np.float32,copy=False)
        rings = view['ring'].reshape(-1).astype(np.float32,copy=False) if 'ring' in names else np.full(count,-1,dtype=np.float32)
    else:
        xyz = np.empty((0,3),dtype=np.float32);rings = np.empty(0,dtype=np.float32)
    distance2 = np.einsum('ij,ij->i',xyz,xyz)
    good = np.isfinite(xyz).all(axis=1) & np.isfinite(rings) & (distance2 > .0001) & (distance2 <= max_range*max_range)
    xyz,rings = xyz[good],rings[good]
    layers = np.unique(rings)
    if len(layers) > 256:
        raise ValueError('Invalid ring count')
    chosen = []
    # Stratification avoids selecting only one ring in interleaved VLP16 data.
    for ordinal,ring in enumerate(layers):
        candidates = np.flatnonzero(rings == ring)
        budget = maximum//len(layers)+(ordinal < maximum%len(layers))
        if len(candidates) > budget:
            candidates = candidates[np.linspace(0,len(candidates)-1,budget,dtype=np.int64)]
        chosen.append(candidates)
    indices = np.concatenate(chosen) if chosen else np.empty(0,dtype=np.int64)
    packed = np.empty((len(indices),4),dtype='<f4')
    packed[:,:3] = xyz[indices];packed[:,3] = rings[indices]
    metadata = {'frame_id':message.header.frame_id,'input_points':count,'valid_points':len(xyz),
                'display_points':len(packed),'rings':[int(v) for v in layers if v >= 0],
                'max_range_m':max_range,'stride_bytes':16,'payload_bytes':packed.nbytes}
    return packed.tobytes(),metadata


class GpsTrack:
    def __init__(self, maximum=3000):
        self.origin = None
        self.track = deque(maxlen=maximum)

    def update(self, message):
        lat,lon,alt = [finite(getattr(message,k)) for k in ('latitude','longitude','altitude')]
        status = int(message.status.status)
        result = {'fix_status':status,'valid_fix':False,'frame_id':message.header.frame_id}
        if status < 0 or lat is None or lon is None or not -90 <= lat <= 90 or not -180 <= lon <= 180:
            return result
        if self.origin is None:
            self.origin = (lat,lon,alt)
        lat0,lon0,alt0 = self.origin
        # Local tangent approximation for short water-trial tracks.
        north = math.radians(lat-lat0)*6378137.
        longitude_delta = (lon-lon0+180)%360-180
        east = math.radians(longitude_delta)*6378137.*math.cos(math.radians((lat+lat0)/2))
        self.track.append([round(east,3),round(north,3)])
        result.update(valid_fix=True,latitude_deg=lat,longitude_deg=lon,altitude_m=alt,
                      east_m=east,north_m=north,origin={'latitude_deg':lat0,'longitude_deg':lon0},
                      track_enu_m=list(self.track))
        return result


def bounded_preview(message, max_items=40, depth=0):
    """Keep raw topic inspection bounded, including binary camera/packet fields."""
    if depth > 5:
        return '…'
    if hasattr(message,'get_fields_and_field_types'):
        result = {}
        for key in message.get_fields_and_field_types():
            value = getattr(message,key)
            if key == 'data' and not isinstance(value,(str,int,float,bool)):
                result[key] = {'items':len(value),'summary':'바이너리 데이터'}
            else:
                result[key] = bounded_preview(value,max_items,depth+1)
        return result
    if isinstance(message,(str,bool,int)) or message is None:
        return message[:3000] if isinstance(message,str) else message
    if isinstance(message,float):
        return finite(message)
    if hasattr(message,'__len__') and hasattr(message,'__iter__'):
        result = [bounded_preview(v,max_items,depth+1) for _,v in zip(range(max_items),message)]
        return {'values':result,'total_items':len(message)} if len(message)>max_items else result
    return str(message)[:300]
