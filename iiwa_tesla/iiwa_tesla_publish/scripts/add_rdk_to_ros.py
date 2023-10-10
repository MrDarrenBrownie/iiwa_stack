from robodk import robolink

# instatiate RoboDK object
RDK = robolink.Robolink()

robot = RDK.Item('', itemtype=robolink.ITEM_TYPE_ROBOT)

home_position = RDK.Item('home', itemtype=robolink.ITEM_TYPE_TARGET)

