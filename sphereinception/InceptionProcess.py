import numpy as np
import argparse
import sys,os
import tensorflow as tf
	
class InceptionProcess:
	#Go down 2 levels
	pth = os.path.split(os.path.split(os.getcwd())[0])[0] + "/sphereinception"

	def get_labels(self):
    		#retrieve labels for classification
		d = self.pth +'/retrained_labels.txt'
    		with open(d, 'r') as fin:
        		labels = [line.rstrip('\n') for line in fin]
        		return labels
	def loadGraph(self):
		# Unpersists graph from file
		e = self.pth + "/retrained_graph.pb"
	    	with tf.gfile.FastGFile(e, 'rb') as fin:
			graph_def = tf.GraphDef()
			graph_def.ParseFromString(fin.read())
			_ = tf.import_graph_def(graph_def, name='')

	def predictDigit(self, sess, softmax_tensor, labels, image):
		# Read in the image_data
		image_data = tf.gfile.FastGFile(image, 'rb').read()

		try:
		    	predictions = sess.run(softmax_tensor, {'DecodeJpeg/contents:0': image_data})
		    	prediction = predictions[0]
		except:
		    	print("Error making prediction.")
		    	sys.exit()

		# List of predictions. See retrained_labels.txt for labels.
		
		prediction = prediction.tolist()
		max_value = max(prediction)
		max_index = prediction.index(max_value)
		predicted_label = labels[max_index]
		#print(predicted_label)
		return prediction

	def initRecog(self):
		#predict the image of a class
	       	labels = self.get_labels()
	    	self.loadGraph()
		return labels