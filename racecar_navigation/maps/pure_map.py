#!/usr/bin/python
import cv2
import sys
import numpy as np

def handleImg(img_file, area_threshold):
  src = cv2.imread(img_file)
  im = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
  _, free_space = cv2.threshold(im, 250, 255, cv2.THRESH_BINARY)
  _, occupied = cv2.threshold(im, 0, 10, cv2.THRESH_BINARY_INV)
  _, contours, hierarchy = cv2.findContours(occupied, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

  mask = np.zeros(im.shape, np.uint8)
  max_rect = (0, 0, 0, 0)
  for i in range(len(contours)):
    contours_poly = cv2.approxPolyDP(contours[i], 3, True)
    x, y, w, h = cv2.boundingRect(contours_poly)

    if(w * h > max_rect[2] * max_rect[3]):
      max_rect = (x, y, w, h)
    elif w * h < area_threshold:
      continue

    cv2.drawContours(mask, contours, i, 255, -1)

  cut = np.zeros(im.shape, np.uint8)
  x = max_rect[0]
  y = max_rect[1]
  cut[y:(max_rect[1]+max_rect[3]), x:(max_rect[0]+max_rect[2])] = 255

  im_floodfill = free_space & cut
  im_floodfill[im_floodfill == 255] = 200
  h, w = im_floodfill.shape[:2]
  fmask = np.zeros((h + 2, w + 2), np.uint8)
  cv2.floodFill(im_floodfill, fmask, (w/2, h/2), 255)
  # free_space[im_floodfill != 255] = 0

  background = np.ones(im.shape, np.uint8) * 205
  final = cv2.addWeighted(background, 1.0, free_space, 1.0, 0.0) & (cv2.bitwise_not(mask))

  cv2.imwrite('test.pgm', final)


if __name__ == '__main__':
  handleImg('mymap.pgm', int(sys.argv[1]) if len(sys.argv) > 1 else 100)
