import random
import matplotlib.pyplot as plt

def GenerateRandomScatterPoints():
  '''Generate set of random coordinates based on y = x'''
  inliersTolerance = 2
  inliersCount = 20
  outliersCount = 20
  # list comprehension:
  # [expression for element in iterable if condition ]
  # x - expression - thing to do on each element
  # y - element - name of each element in iterable 
  # z - iterable - the iterable itself
  # w - condition - only perform the expression if the condition is true
  inliers = [
    (i + random.random()*inliersTolerance, 2*(i + random.random()*inliersTolerance)) 
    for i in range(0, inliersCount)
    ]
  outliers = [
    (random.random()*outliersCount, 2*(random.random()*outliersCount))
    for i in range(0, outliersCount)
    ]
  return inliers + outliers

def Ransac2d(
  points, 
  numberOfIterations = 100,
  tolerance = 1):
  '''
  **** RANSAC 2D IMPLEMENTATION ****
  based on https://github.com/divy-works/ransac_tutorial
  '''

  maximum_inlierscount = 0
  coefficients = []

  for iteration in range(0, numberOfIterations):
      index1, index2 = GenerateRandomIndicesTuple(points, size=2)
      # find the coefficients for the line
      # equation of line passing through two points is given by:
      # (y1 -y2)*X + (x2 - x1)*Y + (x1*y2 - y1*x2) = 0
      A = points[index1][1] - points[index2][1]
      B = points[index2][0] - points[index1][0]
      C = points[index1][0]*points[index2][1] - points[index1][1]*points[index2][0]
      
      # find the count of points lying within tolerance of this line
      inlierscount = 0
      for i in range(0, len(points)):
          # perpendicular distance of point x1, y1 from line Ax + By + C = 0 is
          # |Ax1 + By1 + C|/sqrt(A^2 + B^2)
          distance = abs(A*points[i][0] + B*points[i][1] + C)/(pow(A*A + B*B, 0.5))
          if distance < tolerance:
              inlierscount += 1
      if inlierscount > maximum_inlierscount:
          coefficients = [A, B, C]
          maximum_inlierscount = inlierscount
  return coefficients

def TestRansac2d():
  points = GenerateRandomScatterPoints()

  # plot the input points
  for point in points:
      plt.scatter(point[0], point[1], color="blue")
  plt.title('Sample Points for Ransac 2D Algorithm')
  plt.show()

  # run ransac
  coefficients = Ransac2d(points)

  # plot all the points
  for point in points:
      plt.scatter(point[0], point[1], color="green")

  # plot the best fit line
  A = coefficients[0]
  B = coefficients[1]
  C = coefficients[2]
  allxvalues = [point[0] for point in points]
  xvalues = [min(allxvalues), max(allxvalues)]
  yvalues = [(-C-A*xvalues[0])/B, (-C-A*xvalues[1])/B]
  plt.plot(xvalues, yvalues, color="red")
  plt.title('Line Fitted through points in 2D using Ransac')
  plt.show()
  return


  