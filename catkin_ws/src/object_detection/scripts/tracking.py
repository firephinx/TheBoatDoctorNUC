import tensorflow as tf 
import numpy as np 
import cv2



class ActuatorClassifier(object):
    def __init__(self):
        self.path='/home/theboatdoctor-nuc/Desktop/deploy/valves_900_day/frozen_inference_graph.pb'
        PATH_TO_MODEL = self.path#'frozen_inference_graph.pb'
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            # Works up to here.
            with tf.gfile.GFile(PATH_TO_MODEL, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            self.d_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            self.d_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.d_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_d = self.detection_graph.get_tensor_by_name('num_detections:0')
        self.sess = tf.Session(graph=self.detection_graph)



    def get_classification(self, img):
        # Bounding Box Detection.
        with self.detection_graph.as_default():
            # Expand dimension since the model expects image to have shape [1, None, None, 3].
            img_expanded = np.expand_dims(img, axis=0)  
            (boxes, scores, classes, num) = self.sess.run(
                [self.d_boxes, self.d_scores, self.d_classes, self.num_d],
                feed_dict={self.image_tensor: img_expanded})
        return boxes, scores, classes, num


if __name__=='__main__':
    human=ActuatorClassifier()
    img=cv2.imread("/home/foredawnlin/data/mechatroics_valves/2300.jpg")
    #print np.shape(img)
    a,b,c,d=human.get_classification(img)
    #print a,b,c,d
    print "box",a 
    print "scores",b 
    print "classes",c 
    print "num",d 

    height,width,_=np.shape(img)
    print width,height
    for i in np.arange(0,3):
        y=int(a[0][i][0]*height)
        x=int(a[0][i][1]*width)
        h=int((a[0][i][2]-a[0][i][0])*height)
        w=int((a[0][i][3]-a[0][i][1])*width)
        print x,w,y,h
        img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
    cv2.imshow("bbox",img)
    cv2.waitKey(0)