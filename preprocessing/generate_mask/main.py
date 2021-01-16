"""
Generates mask maps out of the segmentated objects represented by polygon corners of MRCNN
"""
import cv2
import numpy as np
import os

BASEPATH = '../../../../../datasets/freiburg3_walking_xyz/results'
SAVE_DIR = os.path.join(BASEPATH, '..', 'rcnn_labelmap_3dmatched')

def extract_vertices(filepath):
  f = open(filepath, "r")
  txt = f.read().split('\n')
  txt = [vert.split(' ') for vert in txt]

  vertices = []
  for row in txt[:-1]:
    vertice = []
    for val in row:
      if val != '':
        vertice.append(float(val))
    vertices.append(np.array(vertice))
  return vertices

if __name__ == "__main__":
    all_files = os.listdir(BASEPATH)
    filenames = []
    for file in all_files:
        if 'obj_vertices' in file:
            filenames.append(file)
    filenames = sorted(filenames)
    filecount = len(filenames)

    for j, filename in enumerate(filenames):
        filepath = os.path.join(BASEPATH, filename)
        img = np.zeros((480, 640))
        vertices = extract_vertices(filepath)

        for i in range(0, int(len(vertices) / 2)):
            ppt = np.array([list(zip(vertices[i * 2 + 1],vertices[i * 2]))], dtype=np.int32)
            cv2.fillPoly(img, np.array([ppt], dtype=np.int32), color=(2, 2, 2), lineType=8)
        cv2.imwrite(os.path.join(SAVE_DIR, f'{j}_maskmap.png'), img)
        
        if j % 50 == 0:
            print(f'{j}/{filecount}')