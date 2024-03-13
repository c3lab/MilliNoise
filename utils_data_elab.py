#!/usr/bin/env python3
import threading
import pickle
import numpy as np

'''
 Obstacles Dimensions
    Length x Height x Depth
    - Std Obstacle: 48x35x25 (cm)
    - Medium Obstacle: (48x3)x35x25 (cm)
    - Long Obstacle: (48x4)x35x25 (cm)
    - Monitor Obstacle: 95x23x45 (cm)
'''

obs_topics = [
    "MediumObstacle",
    "LongObstacle",
    "ObstacleMonitor",
    "Obstacle",
]

STD_OBS = [0.49, 0.36, 0.25]
MED_OBS = [0.36, 1.47,0.25]
LON_OBS = [1.96, 0.36, 0.25]
MON_OBS = [0.23, 0.95, 0.45]

obs_info = {
    obs_topics[0]: MED_OBS,
    obs_topics[1]: LON_OBS,
    obs_topics[2]: MON_OBS,
    obs_topics[3]: STD_OBS,
}


VIC_OFF_X = 3
VIC_OFF_Y = 2.7

class run():
    
    def __init__(self,j,scene_select_ID,s,n_obs,obs_id):
        self.ID_run=j
        self.s_ID=scene_select_ID
        self.scene_selected=s
        self.data_camera_image=[]
        self.data_pose=[]
        self.data_mmwave=[]
        self.data_laser=[]
        self.data_imu=[]
        self.data_camera_depth=[]
        self.timestamp=[]
        for i in range (0,n_obs): 
            self.scene_selected.obs[i]=obstacle(self.scene_selected.obs[i].l1,self.scene_selected.obs[i].l2,i,obs_id)
        self.init_pose=0   
        self.goal_pose=[0,0]
        print("self.init_pose= ",self.init_pose)
        print("self.goal_pose= ",self.goal_pose)


    def __str__(self):
        return str(self.__class__)+":"+str(self.__dict__)

    def vicon_odom(self): 
        pass
    
    def vicon_odomCB(self,msg):
        data_pose_x = round(msg.transform.translation.x,4)+3
        data_pose_y = round(msg.transform.translation.y,4)+2.7
        theta = round(0,4) 
        self.data_pose_last_value=[data_pose_x, data_pose_y, theta]

    def lidar(self): 
        pass
    
    def lidarCB(self,msg):
        self.data_laser_last_value=msg.ranges

    def imu6dof(self): 
        pass

    def imuCB(self,msg):
        data_imu_w=np.array([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
        data_imu_l_acc=np.array([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
        self.data_imu_last_value=np.concatenate((data_imu_w, data_imu_l_acc), axis=None) #restituisce un array di 6 elementi (3 per vel angolare, 3 per acc lineare)
    
    def camera_image(self): 
        pass

    def camera_imageCB(self,msg):
        self.data_camera_image_last_value=msg

    def camera_depth(self): 
        pass
    
    def camera_depthCD(self,msg):
        self.data_camera_depth_last_value=msg

    def mmWave(self): 
        pass

    def mmWaveCB(self,msg):
        global data_mmwave_last_value
        global lock

    def random_goal_pose(self,n_obs):
        tol=0.2 
        goal_x=np.random.uniform(0+tol,2*3-tol)
        goal_y=np.random.uniform(0+tol,2*2.7-tol)
        theta=np.random.uniform(-np.pi,np.pi) 
        while any(self.goal_true(goal_x,goal_y,n_obs)): 
            goal_x=np.random.uniform(0,2*3)
            goal_y=np.random.uniform(0,2*2.7)

        return [goal_x,goal_y,theta]

    def goal_true(self,x,y,n_obs):
        tol=0.2 
        g=[]
        dist_from_init_pose_to_goal=np.sqrt((abs(self.init_pose[0])-abs(x))**2+(abs(self.init_pose[1])-abs(y))**2) 
        print("distanza dal goal =",dist_from_init_pose_to_goal)
        for i in range(0,n_obs):
            xg=x-self.scene_selected.obs[i].data_pose_obj[0] 
            yg=y-self.scene_selected.obs[i].data_pose_obj[1]
            xg,yg=0,0
            if (abs(xg)<=(self.scene_selected.obs[i].l1/2)+tol and abs(yg)<=(self.scene_selected.obs[i].l2/2)+tol):
                g.append(True)
            else:
                if 3<dist_from_init_pose_to_goal<4.5:
                        g.append(False)
                else:    
                    g.append(True)
        return g
    
    def get_quaternion(self, theta):
        return np.array([1,0,0,0])

    def set_init_pose(self):
        initpose = None
        initpose.header.stamp = rospy.Time.now()
        initpose.header.frame_id = "map"
        theta=self.init_pose[2]
        initpose.pose.pose.position.x = self.init_pose[0]
        initpose.pose.pose.position.y = self.init_pose[1]
        quaternion = self.get_quaternion(theta)
        initpose.pose.pose.orientation.w = quaternion[0]
        initpose.pose.pose.orientation.x = quaternion[1]
        initpose.pose.pose.orientation.y = quaternion[2]
        initpose.pose.pose.orientation.z = quaternion[3]
        initpose.pose.covariance=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        return initpose
        
    

    def ActionClient_launch(self):
        global manual_enabled
        global data_mmwave_last_value
        global lock
        client = None
        client.wait_for_server()
        goal=MoveBaseGoal()
        goal.target_pose.header.frame_id="/turtlebot3/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position =  Point(self.goal_pose[0],self.goal_pose[1],0)
        theta=self.goal_pose[2]
        q=self.get_quaternion(theta)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rate = 2.0 
        i=0 
        format1='rgb8' 
        format2='passthrough' 
        timeout = 500 
        
        init_time=0
        input(" Premi invio per iniziare la registrazione dei dati ")
        print("Inizio acquisizione dati")
        data_mmwave_last_value = []
        while True:
            try:
                dir_image=os.path.join(os.getcwd(),'data/run/run'+str(self.ID_run)+'/Image')
                self.data_camera_image.append(dir_image+'/run'+str(self.ID_run)+'_frame'+str(i)+'.png')
                dir_image_depth=os.path.join(os.getcwd(),'data/run/run'+str(self.ID_run)+'/Image_depth')
                self.data_camera_depth.append(dir_image_depth+'/run'+str(self.ID_run)+'_framedepth'+str(i)+'.png')
                
                self.data_laser.append(self.data_laser_last_value)
                self.data_imu.append(self.data_imu_last_value)
                self.data_pose.append(self.data_pose_last_value)
                
                self.data_mmwave.append(data_mmwave_last_value)
                data_mmwave_last_value= []
                i+=1
                rate.sleep()
            except KeyboardInterrupt:
                print(" KeyboardInterrupt detected. Interrupting Gracefully.")
                break
            except Exception as e:
                print("Exception detected. Interrupting Gracefully.")
                print(e)
                break

        self.run_total_time=0
        print("Fine acquisizione dati run")  

    def moveToGoal(self):
        uuid = None
        map_selected='map_file:=/home/walter/turtlebot3_ws/src/turtle/script/data/scene/maps/map'+str(self.s_ID) +'.yaml'

        in_x='init_x:='+str(self.init_pose[0])
        in_y='init_y:='+str(self.init_pose[1])
        in_yaw='init_yaw:='+str(self.init_pose[2])
        cli_args1 = ['turtle', 'nav.launch', map_selected,in_x,in_y,in_yaw]
        roslaunch_file1 = None
        roslaunch_args1 = cli_args1[2:]
        launch_files = [(roslaunch_file1[0], roslaunch_args1)]
        parent = None
        parent.start()
        
        self.ActionClient_launch()

        parent.shutdown()

class obstacle():
    def __init__(self, obstacle_topic):
        self.topic=obstacle_topic
        for topic in obs_info.keys():
            if topic in self.topic:
                self.l1 = obs_info[topic][0]
                self.l2 = obs_info[topic][1]
                self.l3 = obs_info[topic][2]
                break
        thread_o = threading.Thread(target = self.vicon_odom)
        thread_o.setDaemon(True)
        thread_o.start()
        self.data_pose_obj = [0,0,0]
        
    def __str__(self):
        return str(self.__class__)+":"+str(self.__dict__)

    def vicon_odom(self):
        pass

class scene():
    def __init__(self,k,obs_id): 
        self.obs=[]
        self.ID_scene=k
        self.num_obstacle=len(obs_id)
        for i in range (0,self.num_obstacle):
            l1=0.36
            l2=0.49
            self.obs.append(obstacle(l1,l2,i,obs_id))
            print(self.obs[i].data_pose_obj)
            
    def __str__(self):
        return str(self.__class__)+":"+str(self.__dict__)


def get_scenes(scene_file):
    scenes = []
    with open(scene_file, "rb") as fscenes:
        while True:
            try:
                scenes.append(pickle.load(fscenes))
            except EOFError:
                break
    return scenes

def get_obs_info_old(scene):

    dims = np.array([np.array([x.l1/2, x.l2/2]) for x in scene.obs])

    coords = np.array([x.data_pose_obj for x in scene.obs])
    coords[:, 0] -= VIC_OFF_X
    coords[:, 1] -= VIC_OFF_Y

    obs_range_x = np.column_stack((np.subtract(coords[:, 0], dims[:, 0]), np.add(coords[:, 0], dims[:, 0])))
    obs_range_y = np.column_stack((np.subtract(coords[:, 1], dims[:, 1]), np.add(coords[:, 1], dims[:, 1])))

    return obs_range_x, obs_range_y, dims

def get_obs_info(scene):
    pos = []
    dims = []
    tops = []
    for obs in scene.obs:
        pos.append(obs.data_pose_obj)
        for top in obs_info.keys():
            if top in obs.topic:
                dims.append(obs_info[top])
                tops.append(obs.topic)
                break

    return pos, dims, tops


def get_scene_index(dataset, base_path="data/"):
    with open(base_path+dataset+"/data_"+dataset.replace('_', ''), "rb") as t:
        try:
            data_run = pickle.load(t)
        except EOFError as e:
            print(e)
    return data_run.scene_selected.ID_scene