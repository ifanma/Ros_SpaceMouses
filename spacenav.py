#!/usr/bin/env python
#coding=utf-8
import usb.core
import usb.util
from time import gmtime, strftime
import signal
import threading
import rospy
from sensor_msgs.msg import Joy

class dev_3d(object):
    def __init__(self):
        self.dev = None
        self.ep_in = None
        self.ep_out = None
        self.reattach = False
        self.data = [0, 0, 0, 0, 0, 0, 0, 0]
        self.id = -1


def dealdata(data_new, data_rec):
    if data_new[0] == 1:
        # translation packet
        data_rec[0] = data_new[1] + (data_new[2]*256)
        data_rec[1] = data_new[3] + (data_new[4]*256)
        data_rec[2] = data_new[5] + (data_new[6]*256)
        
        if data_new[2] > 127:
            data_rec[0] -= 65536
        if data_new[4] > 127:
            data_rec[1] -= 65536
        if data_new[6] > 127:
            data_rec[2] -= 65536
        

    if data_new[0] == 2:
        # rotation packet
        data_rec[3] = data_new[1] + (data_new[2]*256)
        data_rec[4] = data_new[3] + (data_new[4]*256)
        data_rec[5] = data_new[5] + (data_new[6]*256)
        
        if data_new[2] > 127:
            data_rec[3] -= 65536
        if data_new[4] > 127:
            data_rec[4] -= 65536
        if data_new[6] > 127:
            data_rec[5] -= 65536

    if data_new[0] == 3:
        data_rec[6] = data_new[1] & 0x01
        data_rec[7] = (data_new[1] & 0x02)>>1
    
    return data_rec


def sigint_handler(signal, frame):
    global run
    run = False
    print('Key interrupt')


def read_task(dev):

    j_pub = rospy.Publisher("~joy%d"%dev.id, Joy, queue_size=10)
    j = Joy()
    j.axes.extend([0, 0, 0, 0, 0, 0])
    j.buttons.extend([0, 0])
    
    while run:
        try:
            data = dev.dev.read(dev.ep_in.bEndpointAddress, dev.ep_in.bLength, timeout=2)
            dev.data = dealdata(data, dev.data)

            # print '%d'%dev.id, dev.data[0], dev.data[1], dev.data[2], dev.data[3], dev.data[4], dev.data[5], dev.data[6], dev.data[7] 
            j.axes[0] = dev.data[0]
            j.axes[1] = dev.data[1]
            j.axes[2] = dev.data[2]
            j.axes[3] = dev.data[3]
            j.axes[4] = dev.data[4]
            j.axes[5] = dev.data[5]

            j.buttons[0] = dev.data[6]
            j.buttons[1] = dev.data[7]

            j.header.stamp = rospy.Time().now()

            j_pub.publish(j)

        except usb.core.USBError as e:
            # print("USB error%s" % e)
            pass
        except Exception as e:
            print("read failed %s" %e)
        
    usb.util.dispose_resources(dev.dev)

    if dev.reattach:
        dev.dev.attach_kernel_driver(0)

if __name__ == '__main__':
    global run 
    run = True

    signal.signal(signal.SIGINT, sigint_handler)
    rospy.init_node("spacenav")

    # Look for SpaceNavigator
    dev_it = usb.core.find(find_all=True, idVendor=0x256f, idProduct=0xc635)
    dev_list = []
    if dev_it is None:
        raise ValueError('SpaceNavigator not found')
    else:
        print ('SpaceNavigator found')

    threads = []

    for i in dev_it:
        dev_list.append(dev_3d())
        len = dev_list.__len__() - 1
        dev_list[len].dev = i
        
        # Don't need all this but may want it for a full implementation

        # msg = str(dev_list[len].dev)
        # print(msg)
        # bus = msg[msg.find('Bus') + 4: msg.find('Bus') + 7]
        # addr = msg[msg.find('Address') + 8: msg.find('Address') + 11]
        # print(usb.core.Device.serial_number)
        # cfg = dev_list[len].dev.get_active_configuration()
        # print 'cfg is ', cfg
        # intf = cfg[(0,0)]
        # print 'intf is ', intf
        # ep = usb.util.find_descriptor(intf, custom_match = lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)
        # print 'ep is ', ep

        dev_list[len].reattach = False
        if dev_list[len].dev.is_kernel_driver_active(0):
            dev_list[len].reattach = True
            dev_list[len].dev.detach_kernel_driver(0)
        dev_list[len].ep_in = dev_list[len].dev[0][(0,0)][0]
        dev_list[len].ep_out = dev_list[len].dev[0][(0,0)][1]

        dev_list[len].id = len

        t = threading.Thread(target=read_task,args=(dev_list[len],))
        threads.append(t)
        
    print('Important! Exit by pressing Ctrl-C, total %d device(s)' % dev_list.__len__())

    for t in threads:
        t.setDaemon(True)
        t.start()

    while not rospy.is_shutdown():
        rospy.spin()
    print('exit')

    for t in threads:
        t.join()
    