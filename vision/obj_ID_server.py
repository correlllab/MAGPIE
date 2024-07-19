########## INIT ####################################################################################

##### Imports #####
### Standard ###
import time, logging, ctypes, os, sys, traceback, multiprocessing
from time import sleep
from ast import literal_eval
from pprint import pprint
from multiprocessing import Process, Array, Value, RawArray, Manager

### Special ###
import numpy as np
from PIL import Image
import open3d as o3d
from open3d.web_visualizer import draw
import matplotlib.pyplot as plt

### Local ###
sys.path.append( "../" )
from magpie import grasp as gt
from magpie.perception import pcd
from magpie import realsense_wrapper as real
from magpie.perception.label_owlvit import LabelOWLViT



########## PERCEPTION SETTINGS #####################################################################

_QUERIES     = ["a photo of a purple block", "a photo of a blue block", "a photo of a red block"]
_ABBREV_Q    = ["purple", "blue", "red"]
_NUM_BLOCKS  = 3
_PLOT_BOX    = False
_VIZ_PCD     = False
_ID_PERIOD_S = 2.0
_STR_MAX     = 2048



########## PERCEPTION SERVER, MULTIPROCESSING ######################################################


##### Server ##############################################################
# Configure logging
logging.basicConfig( level = logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')



class Perception_OWLViT:
    """ Perception service based on OWL-ViT """

    ### Class Vars ###

    query           = _QUERIES
    abbrevq         = _ABBREV_Q
    blocks          = _NUM_BLOCKS
    visualize_boxes = _PLOT_BOX
    view_pcd        = _VIZ_PCD
    rsc             = None
    label_vit       = None 
    lst             = 0.0 #------------------------------------------ Last time loop ended,  INSIDE  Process
    per             = _ID_PERIOD_S #--------------------------------- Period [s], __________ INSIDE  Process
    effPose         = Array( 'd', np.eye(4).reshape( (16,) ).tolist() )

    # objIDstr        = Array( 'c', bytes( "Hello, World!", 'utf-8' ) )
    # objIDstr        = Array( 'c_wchar_p', "Hello, World!" )
    objIDstr        = Array( ctypes.c_char, [b'\x00' for _ in range( _STR_MAX )] )
    # objIDstr        = RawArray( ctypes.c_char, [b'\x00' for _ in range( _STR_MAX )] )
    # objIDstr        = Array( ctypes.c_byte, [0 for _ in range( _STR_MAX )] )
    # objIDstr        = Manager().list( [b'\x00' for _ in range( _STR_MAX )] )

    # objIDstr        = Value( ctypes.c_char_p, b"Hello, World!" )
    # objIDstr        = Value( ctypes.c_wchar_p, "Hello, World!" )

    run             = Value( 'B', 1 )
    viz             = Value( 'B', 1 )

    @classmethod
    def start_vision( cls ):

        # cls.effPose  = Array( 'd', np.eye(4).reshape( (16,) ).tolist() )
        # cls.objIDstr = Array( ctypes.c_char, [b'\x00' for _ in range( _STR_MAX )] )

        try:
            cls.rsc = real.RealSense()
            cls.rsc.initConnection()
            # logging.info( f"RealSense camera CONNECTED" )
            print( f"RealSense camera CONNECTED", flush=True )
        except Exception as e:
            # logging.error( f"Error initializing RealSense: {e}" )
            print( f"\nERROR initializing RealSense: {e}\n", flush=True )
            raise e
        
        try:
            cls.label_vit = LabelOWLViT( pth = "google/owlvit-base-patch32" )
            # logging.info( f"V-LLM STARTED" )
            print( f"V-LLM STARTED", flush=True )
        except Exception as e:
            # logging.error( f"Error initializing OWL-ViT: {e}" )
            print( f"\nERROR initializing OWL-ViT: {e}\n", flush=True )
            raise e

    @classmethod
    def transform_point_cloud(cls, cpcd):
        """Transforms the given point cloud to align with the gripper pose."""
        rotation_matrix = np.array([
            [0, 1, 0, 0],
            [-1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        tmat_gripper = np.array([
            [1, 0, 0, -1.15 / 100],
            [0, 1, 0, 1.3 / 100],
            [0, 0, 1, (309.63 - 195.0) / 1000],
            [0, 0, 0, 1]
        ])

        cpcd.transform(tmat_gripper)
        cpcd.transform(rotation_matrix)
        #convert to 4*4
        cpcd.transform( np.asarray( cls.effPose[:] ).reshape( (4,4,) ) )

        return cpcd
    
    @classmethod
    def save_point_cloud(cls, filename, point_cloud):
        """Saves the point cloud to a file."""
        o3d.io.write_point_cloud(filename, point_cloud)

    @classmethod
    def get_pcd_pose(cls, point_cloud):
        """Gets the pose of the point cloud."""
        center = point_cloud.get_center()
        pose_vector = [center[0], center[1], center[2], 3.14, 0, 0]
        return pose_vector

    @classmethod
    def calculate_area(cls, box):
        """Calculates the area of the bounding box."""
        return abs(box[3] - box[1]) * abs(box[2] - box[0])

    @classmethod
    def filter_by_area(cls, tolerance, box, total_area):
        """Filters the bounding box by area."""
        area = cls.calculate_area(box)
        return abs(area / total_area) <= tolerance

    @classmethod
    def bound(cls, query, abbrevq):
        """Bounds the given query with the OWLViT model."""
        _, rgbd_image = cls.rsc.getPCD()
        image = np.array(rgbd_image.color)

        cls.label_vit.set_threshold(0.005)
        _, _, scores, labels = cls.label_vit.label(image, query, abbrevq, topk=True, plot=False)

        scores = sorted(scores, reverse=True)
        filtered_boxes = []
        filtered_scores = []
        filtered_labels = []
        filter_coords = []

        for i in range(min(20, len(cls.label_vit.sorted_labeled_boxes_coords))):
            if cls.filter_by_area(0.05, cls.label_vit.sorted_labeled_boxes_coords[i][0], image.shape[0] * image.shape[1]):
                filtered_boxes.append(cls.label_vit.sorted_boxes[i])
                filtered_scores.append(scores[i])
                filtered_labels.append(labels[i])
                filter_coords.append(cls.label_vit.sorted_labeled_boxes_coords[i])

        return rgbd_image, image, abbrevq, filtered_boxes, filtered_scores, filtered_labels, filter_coords

    @classmethod
    def calculate_probability_dist(cls, cluster):
        """Calculates the probability distribution of the cluster."""
        probabilities = {color: 0 for color in _ABBREV_Q}
        total = len(cluster)
        color_counts = {color: 0 for color in _ABBREV_Q}

        for _, color, _, _ in cluster:
            color_counts[color] += 1

        for key in color_counts.keys():
            probabilities[key] = color_counts[key] / total

        return probabilities

    @classmethod
    def format_coordinates(cls, box):
        """Formats the coordinates of the bounding box."""
        cx, cy, w, h = box
        return [cx - w / 2, cy + h / 2, cx + w / 2, cx - h / 2]

    @classmethod
    def find_euclidean_distance(cls, point1, point2):
        """Finds the Euclidean distance between two points."""
        return np.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)

    @classmethod
    def plot_bounding_boxes(cls, input_image, scores, boxes, labels, topk=False, show_plot=False):
        """Plots the bounding boxes on the input image."""
        fig, ax = plt.subplots(1, 1, figsize=(8, 8))
        ax.imshow(input_image, extent=(0, 1, 1, 0))
        ax.set_axis_off()
        idx = 0

        for i in range(_NUM_BLOCKS):
            for score, box, label in zip(scores[i], boxes[i], labels[i]):
                if score < cls.label_vit.SCORE_THRESHOLD:
                    continue
                cx, cy, w, h = box
                ax.plot([cx - w / 2, cx + w / 2, cx + w / 2, cx - w / 2, cx - w / 2],
                        [cy - h / 2, cy - h / 2, cy + h / 2, cy + h / 2, cy - h / 2], "r")
                ax.text(
                    cx - w / 2,
                    cy + h / 2 + 0.015,
                    f"({idx}): {score:1.2f}",
                    ha="left",
                    va="top",
                    color=_ABBREV_Q[i],
                    bbox={
                        "facecolor": "white",
                        "edgecolor": "red",
                        "boxstyle": "square,pad=.3"
                    })
                idx += 1
            
            fig.canvas.draw()
            if not show_plot:
                plt.close(fig)

    @classmethod
    def assign_label(cls, k, boundaries):
        """Assigns a label based on the index."""
        output_list = [sum(boundaries[:i + 1]) - 1 for i in range(len(boundaries))]
        for i in range(len(output_list)):
            if k <= output_list[i]:
                return i

    @classmethod
    def find_clusters(cls, boxes, scores_list):
        """Finds clusters in the bounding boxes."""
        bounding_boxes = []
        boundaries = []
        scores = []

        for i in range(len(boxes)):
            boundaries.append(len(boxes[i]))
            bounding_boxes.extend(boxes[i])
            scores.extend(scores_list[i])

        minw = float('inf')
        for box in bounding_boxes:
            if box[2] < minw:
                minw = box[2]

        clustered_objects = []
        while True:
            clusters = []
            temp = None

            for i in range(len(bounding_boxes)):
                if bounding_boxes[i] is not None:
                    if not clusters:
                        temp = bounding_boxes[i]
                        k = cls.assign_label(i, boundaries)
                        clusters.append([bounding_boxes[i], _ABBREV_Q[k], scores[i], i])
                        bounding_boxes[i] = None
                    else:
                        if cls.find_euclidean_distance([temp[0], temp[1]], [bounding_boxes[i][0], bounding_boxes[i][1]]) < minw:
                            k = cls.assign_label(i, boundaries)
                            clusters.append([bounding_boxes[i], _ABBREV_Q[k], scores[i], i])
                            bounding_boxes[i] = None

            clustered_objects.append(clusters)
            if all(x is None for x in bounding_boxes) or len(clusters) == 1:
                break

        return clustered_objects

    @classmethod
    def build_model(cls):
        """Builds the perception model."""

        print( f"\nInside `build_model`\n", flush=True )

        try:
            images, abbrevqs, filtered_boxes, filtered_scores, filtered_labels, filtered_coords = [], [], [], [], [], []

            for j in range(len(_QUERIES)):
                rgbd, image, abbrevq, boxes, scores, labels, coords = cls.bound(_QUERIES[j], _ABBREV_Q[j])
                
                abbrevqs.append(abbrevq)
                filtered_boxes.append(boxes)
                filtered_scores.append(scores)
                filtered_labels.append(labels)
                filtered_coords.append(coords)

            print( f"\nAbout to build clusters ...\n", flush=True )

            clusters = cls.find_clusters(filtered_boxes, filtered_scores)
            objIDstrTemp = {f'Object {objectnum + 1}': {} for objectnum in range(len(clusters))}

            index_to_segment = [max(cluster, key=lambda x: x[2])[3] for cluster in clusters]
            formatted_boxes = [box for box_list in filtered_coords for box in box_list]

            print( f"\nIs there a depth image?: {rgbd}\n", flush=True )
            print( f"\nAre there boxes?: {formatted_boxes}\n", flush=True )

            for num, index in enumerate(index_to_segment):

                print( f"\nNumber {num}, Index {index}, Camera {cls.rsc} ...\n", flush=True )

                cpcd = None

                try:
                    _, cpcd, _, _ = pcd.get_segment(
                        formatted_boxes,
                        index,
                        rgbd,
                        cls.rsc,
                        type="box",
                        method="iterative",
                        display=False,
                        viz_scale=1000
                    )
                    print( f"\nPCD {cpcd} ...\n", flush=True )
                except Exception as e:
                    # logging.error(f"Error building model: {e}", flush=True)
                    print(f"Segmentation error: {e}", flush=True)
                    raise e
                
                try:
                    print( f"\nAbout to transform PCD ...\n", flush=True )

                    cpcd = cls.transform_point_cloud( cpcd )
                    if cls.view_pcd:
                        cls.visualize_point_cloud( cpcd )
                except Exception as e:
                    # logging.error(f"Error building model: {e}", flush=True)
                    print(f"Transformation error: {e}", flush=True)
                    raise e

                objIDstrTemp[f'Object {num + 1}']['Probability'] = cls.calculate_probability_dist(clusters[num])
                objIDstrTemp[f'Object {num + 1}']['Pose'] = cls.get_pcd_pose(cpcd)

            if cls.visualize_boxes:
                cls.plot_bounding_boxes(image, filtered_scores, filtered_boxes, filtered_labels, topk=False, show_plot=True)

                
            # cls.objIDstr.value = bytes(objIDstrTemp , 'utf-8')
            return objIDstrTemp

        except Exception as e:
            # logging.error(f"Error building model: {e}", flush=True)
            print(f"Error building model: {e}", flush=True)
            raise e
        
        except KeyboardInterrupt as e:
            # logging.error( f"Program was stopped by user: {e}", flush=True )
            print( f"Program was stopped by user: {e}", flush=True )
            traceback.print_exc()

    @classmethod
    def TARGET_ID_loop( cls ):
        """ Retrieve and store a force reading """
        # NOTE: DO NOT ALLOW OUTSIDE REFERENCE TO ANY OBJECT MORE COMPLEX THAN A SIMPLE TYPE
        
        print( f"Begin process {os.getpid()}", flush=True )

        # 0. Set up perception
        cls.start_vision()
        
        print( f"Init complete!, About to run loop ...", flush=True )

        while cls.run.value:
            # 1. Mark the start of the loop
            t_mark = time.time() 

            print( f"\nRunning process {os.getpid()}\n", flush=True )

            # 2. Fetch the pose + image && identify objects in the scene
            if cls.viz.value:
                print( f"\nVision was ENABLED\n", flush=True )
                data = cls.build_model()

                # cls.objIDstr.value = bytes( str( data ), 'utf-8' )
                # cls.objIDstr.value = ctypes.c_char_p( str( data ).encode() )

                byteArr = str( data ).encode()

                cls.objIDstr.acquire()
                for i, byt in enumerate( byteArr ):
                    cls.objIDstr[i] = byt    
                # cls.objIDstr[:len(byteArr)] = byteArr[:]
                cls.objIDstr[len(byteArr)] = b'\0'
                cls.objIDstr.release()
                
                print( f"\n\nPerception Result: {data}\n", flush=True )
                
                # print( f"\n\nStored: {cls.objIDstr.value.decode()}\n", flush=True )
                print( f"\n\nStored: {cls.objIDstr[:]}\n", flush=True )

            else:
                print( f"\nVision was DISABLED\n", flush=True )

            print( f"\nIteration complete\n", flush=True )
            
            # 3. Determine if there is slack to wait
            elapsed = time.time() - cls.lst
            # 4. Store loop start
            cls.lst = t_mark

            # # 5. Consume slack
            # if elapsed < cls.per:
            #     sleep( cls.per - elapsed )

        print( f"About to KILL {os.getpid()}", flush=True )
        os.system( 'kill %d' % os.getpid()) 


########## O_ID_Classifier, MULTIPROCESSING, PARENT #########################################

class Object_ID_Manager:
    """ Use the OWL-ViT in a separate process to identify objects in a scene in the BG """
    _DEBUG = 1

    def __init__( self ):
        self.instance = Perception_OWLViT()


    def start( self ):
        """ Start the streaming process """
        # Perception_OWLViT.viz.value = bool(1)
        # Perception_OWLViT.run.value = bool(1)
        self.proc = Process( 
            target = Perception_OWLViT.TARGET_ID_loop,  
            # target = self.instance.TARGET_ID_loop,  
            # daemon = True,
            daemon = False,
        )
        self.proc.start()


    def stop( self ):
        Perception_OWLViT.run.value = bool(0)
        sleep( 0.25 )
        self.proc.join(timeout=1)
        if self.proc.is_alive():
            self.proc.kill()
            self.proc.kill()


    def get_current_observation( self ):
        content = None
        try:

            msg = bytearray()
            print( '\n\n' )
            Perception_OWLViT.objIDstr.acquire()
            for byt in Perception_OWLViT.objIDstr[:]:
                print( byt, end=', ', flush=True )
                if byt != b'\x00':
                    msg.append( byt )
                else:
                    break
            Perception_OWLViT.objIDstr.release()
            print( "",flush=True )
            content = msg.decode()
            # content = Perception_OWLViT.objIDstr.value.decode()
            # content = Perception_OWLViT.objIDstr.value
            content = bytes( Perception_OWLViT.objIDstr[:] ).decode()

            print( f'\n`get_current_observation`, Decoded message of length {len(content)}: {content}\n' )
            struct  = literal_eval( content )
        except Exception as e:
            print( f"{e}, Could NOT parse!: {content}" )
            struct = list()
        return struct



if __name__ == "__main__":

    # https://github.com/isl-org/Open3D/issues/4007#issuecomment-940996106
    multiprocessing.set_start_method( 'forkserver', force = True )

    print( "Create mamanger ..." )
    mngr = Object_ID_Manager()
    print( "Start thread ..." )
    mngr.start()
    
    sleep( 20 )

    for i in range( 10 ):
        pprint( mngr.get_current_observation() )
        print( '\n' )
        sleep(2)

    print( "Stop thread ..." )
    mngr.stop()
    print( "DONE" )
    os.system( 'kill %d' % os.getpid() ) 