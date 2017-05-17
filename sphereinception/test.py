import sys
import tensorflow as tf
from InceptionProcess import InceptionProcess
imager = InceptionProcess()
if __name__ == '__main__':
    imager.predict_on_image("/home/blueplum/tf_files/DIGITS/1/00000385.jpg")
