import rospy
from gazebo_msgs.srv import GetWorldProperties

def get_model_names():
    rospy.init_node('get_model_names', anonymous=True)
    rospy.wait_for_service('/gazebo/get_world_properties')
    try:
        get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        world_properties = get_world_properties()
        return world_properties.model_names
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    model_names = get_model_names()
    if model_names:
        print("Model names in Gazebo:")
        for name in model_names:
            print(name)
    else:
        print("No models found in Gazebo.")
