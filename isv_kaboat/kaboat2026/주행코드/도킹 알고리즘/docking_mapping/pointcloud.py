"""Decode selected scalar fields without requiring every cloud field to match."""
import numpy as np
from sensor_msgs_py import point_cloud2


def read_scalar_fields(cloud, field_names):
    names=tuple(field_names)
    if not names:raise ValueError('At least one point field is required')
    fields={field.name:field for field in cloud.fields}
    for name in names:
        if name not in fields or fields[name].count!=1:
            raise ValueError('Missing or non-scalar point field: '+name)
    records=point_cloud2.read_points(cloud,field_names=names,skip_nans=True)
    if isinstance(records,np.ndarray) and records.dtype.names is not None:
        result=np.column_stack([records[name].reshape(-1) for name in names]).astype(np.float64,copy=False)
    else:
        rows=list(records)
        result=np.asarray(rows,dtype=np.float64).reshape(-1,len(names)) if rows else np.empty((0,len(names)),dtype=np.float64)
    return result[np.isfinite(result).all(axis=1)]
