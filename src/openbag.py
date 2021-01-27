import rosbag
bag = rosbag.Bag('grid.bag')
for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
    print msg
bag.close()
