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
        #j indica è l'identificativo (numero) della run, scene_select indica la scena selezionata (oggetto classe scena)
        self.ID_run=j
        self.s_ID=scene_select_ID #mi salvo l'ID della scena selezionata come attributo nell'oggetto run che sto creando
        self.scene_selected=s
        self.data_camera_image=[]
        self.data_pose=[]
        self.data_mmwave=[]
        self.data_laser=[]
        self.data_imu=[]
        self.data_camera_depth=[]
        self.timestamp=[]
        #ricreo gli ostacoli nella scena selezionata (senza modificare le scene salvate nel file) in modo tale da avere le posizione 
        #precisa del vicon (al netto delle tolleranze)
        for i in range (0,n_obs): 
            self.scene_selected.obs[i]=obstacle(self.scene_selected.obs[i].l1,self.scene_selected.obs[i].l2,i,obs_id)
        self.init_pose=0   
        self.goal_pose=[0,0]  #funzione con calcolo randomico goal e controllo su intersezione ostacolo
        print("self.init_pose= ",self.init_pose)
        print("self.goal_pose= ",self.goal_pose)
        # self.moveToGoal() #prende (da self) la posizone finale finale calcolata per il navigation stack per fare la navigazione


    #funzione per fare il print degli attributi della classe (print(run))
    def __str__(self):
        return str(self.__class__)+":"+str(self.__dict__)

    #posa del turtlebot
    def vicon_odom(self): 
        # rospy.Subscriber('/vicon/turtlebot3/turtlebot3', SafeVicon, self.vicon_odomCB) 
        # rospy.spin()
        pass
    
    def vicon_odomCB(self,msg):
        data_pose_x = round(msg.transform.translation.x,4)+3
        data_pose_y = round(msg.transform.translation.y,4)+2.7
        theta = round(0,4) 
        self.data_pose_last_value=[data_pose_x, data_pose_y, theta]

    def lidar(self): 
        # rospy.Subscriber('/turtlebot3/scan', sm.LaserScan, self.lidarCB)
        # rospy.spin()
        pass
    
    def lidarCB(self,msg):
        self.data_laser_last_value=msg.ranges

    def imu6dof(self): 
        # rospy.Subscriber('/turtlebot3/imu', sm.Imu, self.imuCB)
        # rospy.spin()
        pass

    def imuCB(self,msg):
        data_imu_w=np.array([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
        data_imu_l_acc=np.array([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
        self.data_imu_last_value=np.concatenate((data_imu_w, data_imu_l_acc), axis=None) #restituisce un array di 6 elementi (3 per vel angolare, 3 per acc lineare)
    
    def camera_image(self): 
        # rospy.Subscriber('/turtlebot3/camera/color/image_raw', sm.Image, self.camera_imageCB)
        # rospy.spin()
        pass

    def camera_imageCB(self,msg):
        self.data_camera_image_last_value=msg

    def camera_depth(self): 
        # rospy.Subscriber('/turtlebot3/camera/depth/image_rect_raw', sm.Image, self.camera_depthCD)
        # rospy.Subscriber('/turtlebot3/camera/aligned_depth_to_color/image_raw', sm.Image, self.camera_depthCD)
        # rospy.spin()
        pass
    
    def camera_depthCD(self,msg):
        self.data_camera_depth_last_value=msg

    def mmWave(self): 
        # rospy.Subscriber('/ti_mmwave/radar_scan_pcl', sm.PointCloud2, self.mmWaveCB)
        # rospy.spin()
        pass

    def mmWaveCB(self,msg):
        global data_mmwave_last_value
        global lock
        # with lock:
        #     assert len(msg.data) > 0, "MMWave Not working."
        #     data_mmwave_last_value.append(ros_numpy.point_cloud2.pointcloud2_to_array(msg)) # pointcloud2_to_xyz_array mi restituisce una lista di punti (lista di liste xyz)
            # data_mmwave_last_value conterrà una lista di liste di punti (lista di liste di liste xyz)

    def random_goal_pose(self,n_obs):
        tol=0.2 #evito gi generare punto sul perimetro esterno del vicon perchè potrebbe perdere l'odometria (vicon non vede il robot)
        goal_x=np.random.uniform(0+tol,2*3-tol) #il goal dovrebbe essere da -vic a vic, però il sdr della mappa è posto il modo tale che non ci siamo valori negativi nelle posizioni 
                                                    #(lo zero del sdr map è posto al limite del quadrante negativo)
        goal_y=np.random.uniform(0+tol,2*2.7-tol)
        theta=np.random.uniform(-np.pi,np.pi) #verificare se theta deve essere generato da 0 a 6.28 o da -3.14 a +3.14
        while any(self.goal_true(goal_x,goal_y,n_obs)): #any restituisce true se c'è almeno un elemento true
            goal_x=np.random.uniform(0,2*3)
            goal_y=np.random.uniform(0,2*2.7)
        # print("DOPO CORREZIONE: Se c'è un True, non va bene e rigenero il goal (tutti false va bene)",self.goal_true(goal_x,goal_y,n_obs))
        return [goal_x,goal_y,theta]

    def goal_true(self,x,y,n_obs):
        tol=0.2 #tolleranza per considerare l'ingombro totale tra turtlebot e ostacolo
        # g=False
        g=[]
        dist_from_init_pose_to_goal=np.sqrt((abs(self.init_pose[0])-abs(x))**2+(abs(self.init_pose[1])-abs(y))**2) #distanza tra goal e pos attuale turtlebot
        print("distanza dal goal =",dist_from_init_pose_to_goal)
        for i in range(0,n_obs):
            xg=x-self.scene_selected.obs[i].data_pose_obj[0]#traslazione e rotazione ostacolo e goal (sdr al centro dell'ostacolo)
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
        initpose = None#PoseWithCovarianceStamped()
        initpose.header.stamp = rospy.Time.now()
        initpose.header.frame_id = "map"
        # print("x=",self.init_pose[0])
        # print("y=",self.init_pose[1])
        # print("theta=",self.init_pose[2])
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
        #define a client to send goal requests to the move_base server through a SimpleActionClient
        client = None#actionlib.SimpleActionClient('turtlebot3/move_base',MoveBaseAction)
        # client.wait_for_server()
        # goal=MoveBaseGoal()
        # goal.target_pose.header.frame_id="/turtlebot3/map"
        # goal.target_pose.header.stamp = rospy.Time.now()
        # goal.target_pose.pose.position =  Point(self.goal_pose[0],self.goal_pose[1],0) # self.goal_pose[0]= x del goal,self.goal_pose[1]= y del goal
        theta=self.goal_pose[2]
        q=self.get_quaternion(theta)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rate = 2.0 #Frequenza di lettura dei dati dai sensori (per non far crashare il pacchetto del mmWave è consigliato acquisire a 2~4Hz)
        i=0 #indice usato solo per far variare il numero di frame salvati (immagini e immagini depth)
        format1='rgb8' # salvare le immagini in rgb
        format2='passthrough' # 'passthrough' o '32FC1' per salvare la depthmap
        timeout = 500 #secondi di timeout per il goal
        
        init_time=0
        input(" Premi invio per iniziare la registrazione dei dati ")
        print("Inizio acquisizione dati")
        data_mmwave_last_value = []
        #condizione di raggiungimento del goal AND condizione durata run inferiore di 70secondi
        while True:
            try:
                # self.timestamp.append(time.time())
                dir_image=os.path.join(os.getcwd(),'data/run/run'+str(self.ID_run)+'/Image')
                self.data_camera_image.append(dir_image+'/run'+str(self.ID_run)+'_frame'+str(i)+'.png')
                # save_image(self.data_camera_image[-1],self.data_camera_image_last_value,format1) #salva le immagini (path,immagine,formato)
                dir_image_depth=os.path.join(os.getcwd(),'data/run/run'+str(self.ID_run)+'/Image_depth')
                self.data_camera_depth.append(dir_image_depth+'/run'+str(self.ID_run)+'_framedepth'+str(i)+'.png')
                # save_image(self.data_camera_depth[-1],self.data_camera_depth_last_value,format2) #salva le immagini (path,immagine,formato)
                
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

        #Dato che per la navigazione serve caricare la mappa quando si lancia la navigazione (mappa che può cambiare ad ogni run), si lancia nav.launch da 
        #codice per caricare ogni volta una mappa diversa (se si lancia all'inizio il nav (scelta più ovvia e funzionale), si è costretti ad utilizzare una 
        #mappa che deve essere nota già dall'inizio)

        #####Launch file nav.launch 
        uuid = None
        map_selected='map_file:=/home/walter/turtlebot3_ws/src/turtle/script/data/scene/maps/map'+str(self.s_ID) +'.yaml'

        #set pos iniziale (serve solo se si usa amcl, con il vicon non serve perchè abbiamo già la localizzazione perfetta)
        #degli arg settati in cli_args1 viene utilizzato solo map_selected per caricare la mappa. in_x,in_y,in_yaw servono per amcl che è commentato nel launch
        in_x='init_x:='+str(self.init_pose[0])
        in_y='init_y:='+str(self.init_pose[1])
        in_yaw='init_yaw:='+str(self.init_pose[2])
        cli_args1 = ['turtle', 'nav.launch', map_selected,in_x,in_y,in_yaw]
        roslaunch_file1 = None
        roslaunch_args1 = cli_args1[2:]
        launch_files = [(roslaunch_file1[0], roslaunch_args1)]
        parent = None
        parent.start()
        #### end launch

        self.ActionClient_launch()

        parent.shutdown()

class obstacle():
    # data_pose_obj=[]
    # h=0
    # r=0
    # ID_obs=0
    def __init__(self, obstacle_topic):
        # obj='Obstacle'+str(i)
        # self.topic='/vicon/'+str(obj)+'/'+str(obj)
        self.topic=obstacle_topic
        for topic in obs_info.keys():
            if topic in self.topic:
                self.l1 = obs_info[topic][0]
                self.l2 = obs_info[topic][1]
                self.l3 = obs_info[topic][2]
                break
        # if 'LongObstacle' in obstacle_topic:
        #     self.l1=l1 # dimensione lungo l'asse x (quella più piccola nel nostro caso)
        #     self.l2=l2*4 # dimensione lungo l'asse y
        # else:
        #     self.l1=l1 # dimensione lungo l'asse x (quella più piccola nel nostro caso)
        #     self.l2=l2 # dimensione lungo l'asse y
        thread_o = threading.Thread(target = self.vicon_odom)
        thread_o.setDaemon(True)
        thread_o.start()
        self.data_pose_obj = [0,0,0]
        # self.vicon_obj
        # print(self.topic)
        # print(self.ID_obs)
        # rospy.sleep(0.1)
        
    def __str__(self):
        return str(self.__class__)+":"+str(self.__dict__)

    def vicon_odom(self):
        # rospy.Subscriber(self.topic, SafeVicon, self.vicon_odomCD)
        # rospy.spin()
        pass


    # def vicon_odomCD(self,msg):
    #     data_pose_x = round(msg.transform.translation.x,4)+vic_x
    #     data_pose_y = round(msg.transform.translation.y,4)+vic_y
    #     theta = round(get_angle(msg.transform.rotation),4) 
    #     self.data_pose_obj=[data_pose_x, data_pose_y, theta]

class scene():
    # obs=[]
    # ID_scene=[]
    # k indica l'ID della scena attuale (lastID+1), scene_l contiene tutte le scene caricate già presenti nel file pickle (derivanti da def read_scene())
    def __init__(self,k,obs_id): #costruendo nuova scena
        self.obs=[]
        self.ID_scene=k
        self.num_obstacle=len(obs_id)
        #k indica il numero della scena (ID)
        for i in range (0,self.num_obstacle):
            l1=0.36 #input('Inserisci dimensione 1 ostacolo '+str(i)+':') #dimensione lungo x
            l2=0.49#input('Inserisci dimensione 2 ostacolo '+str(i)+':') #dimensione lungo y
            self.obs.append(obstacle(l1,l2,i,obs_id))
            print(self.obs[i].data_pose_obj)
            # print("theta ostacolo = ",str(np.rad2deg(self.obs[i].data_pose_obj[2])) +" gradi")
            # print("IN costruttore scena, ostacolo "+str(i)+"= "+str(self.obs[i]))
    #funzione per fare il print degli attributi della classe (print(scene))
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
    # with open(base_path+dataset+"/data_"+dataset, "rb") as t:
    # with open(base_path+dataset.replace('_', '')+"/data_"+dataset.replace('_', ''), "rb") as t:
    with open(base_path+dataset+"/data_"+dataset.replace('_', ''), "rb") as t:
        try:
            data_run = pickle.load(t)
        except EOFError as e:
            print(e)
    return data_run.scene_selected.ID_scene