"""Use a Linux pseudo-terminal, never a hardware serial port."""
import importlib.util
import os
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@unittest.skipUnless(os.name == 'posix' and importlib.util.find_spec('serial'), 'POSIX and pyserial required')
class SerialExclusiveTests(unittest.TestCase):
    def test_same_transport_recovers_missing_port_and_usb_replacement(self):
        import pty, select, tempfile, threading, time
        from isv_water_trials.arduino_bridge import ReconnectingTransport
        from isv_water_trials.actuator_protocol import packet
        boards=[]; events=[]; threads=[]; commands=[]
        def board():
            master,slave=pty.openpty(); done=threading.Event()
            boards.append((master,slave));events.append(done)
            identity=len(boards)
            def firmware():
                buffer=b'';armed=False;seq='0';left=right='1500';last=0
                while not done.is_set():
                    try:
                        if select.select([master],[],[],.01)[0]:buffer+=os.read(master,4096)
                        while b'\n' in buffer:
                            line,buffer=buffer.split(b'\n',1)
                            fields=line.split(b'*')[0].decode().split(',')
                            if len(fields)!=5 or fields[0]!='K1':continue
                            _,seq,op,l,r=fields;commands.append((identity,op,l,r))
                            if op in ('I','S','D'):armed=False;left=right='1500'
                            elif op=='A':armed=True;left=right='1500'
                            elif op=='C' and armed:left,right=l,r
                            os.write(master,packet('S1,%s,2,%d,%s,%s,0,0,0'%(seq,armed,left,right)))
                        if time.monotonic()-last>.05:
                            os.write(master,packet('S1,%s,2,%d,%s,%s,0,0,0'%(seq,armed,left,right)));last=time.monotonic()
                    except OSError:return
            thread=threading.Thread(target=firmware,daemon=True);threads.append(thread);thread.start()
            return os.ttyname(slave)
        def until(predicate):
            end=time.monotonic()+5
            while not predicate() and time.monotonic()<end:time.sleep(.02)
            self.assertTrue(predicate())
        transport=None
        try:
            with tempfile.TemporaryDirectory() as directory:
                port=Path(directory)/'arduino'
                transport=ReconnectingTransport(str(port),lambda message:None,retry_delay=.05,reset_on_connect=False)
                self.assertFalse(transport.exchange('C',1580,1580))
                os.symlink(board(),port)
                until(lambda:transport.snapshot()[0] is not None)
                self.assertTrue(transport.exchange('A'))
                self.assertTrue(transport.exchange('C',1580,1580))
                port.unlink();events[0].set();threads[0].join(1)
                os.close(boards[0][0]);boards[0]=(-1,boards[0][1])
                until(lambda:transport.snapshot()[0] is None)
                self.assertTrue(transport.worker.is_alive())
                self.assertFalse(transport.exchange('C',1580,1580))
                os.symlink(board(),port)
                until(lambda:transport.snapshot()[0] is not None)
                self.assertEqual((transport.snapshot()[0].left,transport.snapshot()[0].right),(1500,1500))
                self.assertFalse(any(row[0]==2 and row[1] in ('A','C') for row in commands))
                self.assertTrue(transport.exchange('A'))
                self.assertTrue(transport.exchange('C',1580,1500))
        finally:
            if transport:transport.close()
            for event in events:event.set()
            for thread in threads:thread.join(1)
            for descriptors in boards:
                for descriptor in descriptors:
                    if descriptor>=0:os.close(descriptor)

    def test_auto_ack_with_receiver_absent(self):
        import pty,select,threading
        from isv_water_trials.arduino_bridge import SerialTransport
        from isv_water_trials.actuator_protocol import packet
        master,slave=pty.openpty();stop=threading.Event();transport=None
        def firmware():
            buffer=b'';armed=False
            while not stop.is_set():
                if not select.select([master],[],[],.02)[0]:continue
                try:buffer+=os.read(master,4096)
                except OSError:return
                while b'\n' in buffer:
                    line,buffer=buffer.split(b'\n',1)
                    parts=line.split(b'*')[0].split(b',')
                    if len(parts)!=5 or parts[0]!=b'K1':continue
                    _,seq,op,left,right=parts
                    armed=op not in (b'D', b'S', b'I')
                    os.write(master,packet('S1,%s,2,%d,%s,%s,0,0,0'%(seq.decode(),armed,left.decode(),right.decode())))
        thread=threading.Thread(target=firmware,daemon=True);thread.start()
        try:
            transport=SerialTransport(os.ttyname(slave))
            self.assertTrue(transport.exchange('D'))
            self.assertTrue(transport.exchange('I'))
            self.assertTrue(transport.exchange('A'))
            self.assertTrue(transport.exchange('C',1540,1540))
            self.assertFalse(transport.snapshot()[0].rc_valid)
            self.assertTrue(transport.exchange('D'))
        finally:
            if transport:transport.close()
            stop.set();thread.join(1);os.close(slave);os.close(master)

    def test_second_bridge_cannot_open_same_device(self):
        import pty
        import serial
        from isv_water_trials.arduino_bridge import SerialTransport
        master, slave = pty.openpty()
        transport = None
        try:
            port = os.ttyname(slave)
            transport = SerialTransport(port)
            with self.assertRaises(serial.SerialException):
                SerialTransport(port)
            self.assertIsNone(transport.snapshot()[2])
        finally:
            if transport is not None:
                transport.close()
            os.close(slave)
            os.close(master)


if __name__ == '__main__':
    unittest.main()
