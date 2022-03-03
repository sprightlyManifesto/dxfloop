import cv2				#apt install python3-ovencv
import dxfgrabber		#pip3 install dxfgrabber
import numpy as np
import math
import matplotlib.pyplot as plt
import os
from decimal import *

class loopElement:
	def __init(self,element,cw):
		self.element = element
		self.cw = cw
		self.normal = None

class profile:
	def __init__(self,filename):
		self.dxf = dxfgrabber.readfile(filename)
		print("DXF version: {}".format(self.dxf.dxfversion)) 
		print("entitles: ",len(self.dxf.entities))
		self.Xmax = 0
		self.Xmin = 9999
		self.Ymax = 0
		self.Ymin = 9999		
		for e in self.dxf.entities:
			if type(e) == dxfgrabber.dxfentities.Arc: 		
				if self.Xmax < (e.center[0] + e.radius): self.Xmax = e.center[0] + e.radius
				if self.Xmin > (e.center[0] - e.radius): self.Xmin = e.center[0] - e.radius
				if self.Ymax < (e.center[1] + e.radius): self.Ymax = e.center[1] + e.radius
				if self.Ymin > (e.center[1] - e.radius): self.Ymin = e.center[1] - e.radius				
			elif type(e) == dxfgrabber.dxfentities.Line:
				if self.Xmin > e.start[0]: self.Xmin = e.start[0]
				if self.Xmin > e.end[0]: self.Xmin = e.end[0]
				if self.Xmax < e.start[0]: self.Xmax = e.start[0]
				if self.Xmax < e.end[0]: self.Xmax = e.end[0]
				if self.Ymin > e.start[1]: self.Ymin = e.start[1]
				if self.Ymin > e.end[1]: self.Ymin = e.end[1]
				if self.Ymax < e.start[1]: self.Ymax = e.start[1]
				if self.Ymax < e.end[1]: self.Ymax = e.end[1]
		print("bouding box: ", self.Xmin , "," , self.Ymin , "    " , self.Xmax , "," , self.Ymax)
	
	def findDxfLoop(self):
		lp = []
		lp.append((self.dxf.entities[0],True))		
		while(self.findNext(lp)):
			continue			
		#print(len(lp))
		print("loop Elements:", len(lp))	
		return lp		

	def findNext(self,ordered):
		found = False
		this = self.startAndEndCartesian(ordered[-1][0])
		#print("typeof: ",type(ordered[-1][0]))
		for e in self.dxf.entities:
			that = self.startAndEndCartesian(e)
			if self.isSamePoint(this[0],that[0]):
				if not((((e,True) in ordered)) or ((e,False) in ordered)):
					ordered.append((e,True))
					found = True
					break
			elif self.isSamePoint(this[0],that[1]):
				if not((((e,True) in ordered)) or ((e,False) in ordered)):
					ordered.append((e,False))
					found = True
					break
			elif self.isSamePoint(this[1],that[0]):
				if not((((e,True) in ordered)) or ((e,False) in ordered)):
					ordered.append((e,True))
					found = True
					break
			elif self.isSamePoint(this[1],that[1]):		
				if not((((e,True) in ordered)) or ((e,False) in ordered)):
					ordered.append((e,False))					
					found = True
					break
		return found

	def isSamePoint(self,p1,p2):
		floatErrorTol = 0.001		
		result = False		
		if(abs(p1[0] - p2[0]) < floatErrorTol) and (abs(p1[1] - p2[1]) < floatErrorTol): 
			result = True
		return result
	
	def startAndEndCartesian(self,e):
		pt= ((0,0),(0,0))
		if type(e) == dxfgrabber.dxfentities.Arc: 
			x1 = -math.sin((e.start_angle-90)*math.pi/180) * e.radius + e.center[0]
			x2 = -math.sin((e.end_angle-90)*math.pi/180) * e.radius + e.center[0]
			y1 = math.cos((e.start_angle-90)*math.pi/180) * e.radius + e.center[1]
			y2 = math.cos((e.end_angle-90)*math.pi/180) * e.radius + e.center[1]
			pt =  ((x1,y1),(x2,y2))
		elif type(e) == dxfgrabber.dxfentities.Line:
			pt = ((e.start[0],e.start[1]),(e.end[0],e.end[1]))
		return pt

	#tuple dxfentitie, CW, ofset vector
	def LoopNormals(self,lp,offset):		
		ln = []	
		for e in lp:
			if type(e[0]) == dxfgrabber.dxfentities.Line:
				start, end = (e[0].start,e[0].end) 								
				vector = (end[0] - start[0], end[1] - start[1])
				mag = math.pow(math.pow(vector[0],2) + math.pow(vector[1],2),0.5)				
				if e[1]:
					ln.append((e[0],e[1],(-vector[1]/mag*offset,vector[0]/mag*offset)))
				else:
					print("backwards:" ,e)
					ln.append((e[0],e[1],(vector[1]/mag*offset,vector[0]/mag*offset)))						

			elif type(e[0]) == dxfgrabber.dxfentities.Arc:
				if e[1]:
					start, end = (self.startAndEndCartesian(e[0])[0], e[0].center)
					vector = (end[0] - start[0], end[1] - start[1])
					mag = math.pow(math.pow(vector[0],2) + math.pow(vector[1],2),0.5)
					ln.append((e[0],e[1],(vector[0]/mag*offset,vector[1]/mag*offset)))
				else:
					start, end = (self.startAndEndCartesian(e[0])[1], e[0].center)
					vector = (end[0] - start[0], end[1] - start[1])
					mag = math.pow(math.pow(vector[0],2) + math.pow(vector[1],2),0.5)
					ln.append((e[0],e[1],(-vector[0]/mag*offset,-vector[1]/mag*offset)))
		return ln

	def MXplusC(self,l):
		m = (l[0].end[1]- l[0].start[1])/(l[0].end[0]- l[0].start[0])
		c =  l[0].start[1] - m*l[0].start[0]					
		return (m,c)

	def comanPoint(self,l1,l2):
		pt = (0,0)
		if self.isSamePoint(l1.start,l2.start):pt = l1.start
		elif self.isSamePoint(l1.start,l2.end):pt = l1.start
		elif self.isSamePoint(l1.end,l2.start):pt = l1.end
		elif self.isSamePoint(l1.end,l2.end):pt = l1.end
		return pt

	def withinFPerror(self,f1,f2):
		fpError = 1e-6		
		result = False	
		if f1 < (f2 + fpError) and f2 < (f1 + fpError): 		
			result = True
		return result

	def intersectLines(self,l1,l2):
		print("intersect: ",l1[0].start[0],l1[0].start[1],l1[0].end[0],l1[0].end[1])
		print("with: ",l2[0].start[0],l2[0].start[1],l2[0].end[0],l2[0].end[1])

		if not(self.withinFPerror(l1[0].start[0],l1[0].end[0])) and not(self.withinFPerror(l1[0].start[1],l1[0].end[1])):
			m1 , c1 = self.MXplusC(l1)
			print("m1: ", m1, " c1: ", c1)
		if not(self.withinFPerror(l2[0].start[0],l2[0].end[0])) and not(self.withinFPerror(l2[0].start[1],l2[0].end[1])):
			m2 , c2 = self.MXplusC(l2)
			print("m2: ", m2, " c1: ", c2)
		
		#l1 vertical
		if self.withinFPerror(l1[0].start[0],l1[0].end[0]):
			x = l1[0].start[0]
			if self.withinFPerror(l2[0].start[0],l2[0].end[0]):
				x, y = self.comanPoint(l1[0],l2[0]) 
			elif self.withinFPerror(l2[0].start[1],l2[0].end[1]):
				y = l2[0].start[1]
			else:				
				y = m2*x + c2				
		
		#l1 horizontial
		elif self.withinFPerror(l1[0].start[1],l1[0].end[1]):
			y = l1[0].start[1]
			if self.withinFPerror(l2[0].start[1],l2[0].end[1]):
				x, y = self.comanPoint(l1[0],l2[0]) 			
			elif l2[0].start[0] == l2[0].end[0]:
				x = l2[0].start[0]
			else:
				x = (y-c2)/m2
				
		#l2 vertical
		elif self.withinFPerror(l2[0].start[0],l2[0].end[0]):
			x = l2[0].start[0]
			if self.withinFPerror(l1[0].start[1],l1[0].end[1]):
				y = l1[0].start[1]
			else:				
				print("l2 Vertical""m1:",m1,"c1:",c1)
				y = m1*x + c1				
		
		#l2 horizontial
		elif self.withinFPerror(l2[0].start[1],l2[0].end[1]):
			y = l2[0].start[1]
			if self.withinFPerror(l1[0].start[0],l1[0].end[0]):
				x = l1[0].start[0]
			else:
				x = (y-c1)/m1

		else:
			x = (c1- c2)/(m2 - m1)						
			y = m1*x + c1
		print("x,y: " ,x,y,"\n")
		return (x,y)
				
	def loopLen(self):
		lenght = 0		
		for e in self.loop:
			if type(e[0]) == dxfgrabber.dxfentities.Line:
				lenght += math.pow((e[0].end[1] - e[0].start[1])**2 + (e[0].end[0] - e[0].start[0])**2,0.5)		
			elif type(e[0]) == dxfgrabber.dxfentities.Arc:
				lenght += e[0].radius * (e[0].end_angle - e[0].start_angle)/ (180/math.pi)
		return lenght

	def offsetLoop(self,offsetLp):
		#offset lines and arcs		
		for e in offsetLp:
			if type(e[0]) == dxfgrabber.dxfentities.Line:
				e[0].start = (e[0].start[0] + e[2][0] , e[0].start[1] + e[2][1])
				e[0].end =   (e[0].end[0] + e[2][0] , e[0].end[1] + e[2][1])
			elif type(e[0]) == dxfgrabber.dxfentities.Arc:
				mag = math.pow(math.pow(e[2][0],2) + math.pow(e[2][1],2),0.5)				
				if e[1]:
					e[0].radius -= mag
				else:
 					e[0].radius += mag
		
		#trim line to correct lenghts so they meet
		for i in range(0,len(offsetLp)):
			prev = offsetLp[i-1]			
			e = offsetLp[i]			
			if i < (len(offsetLp)-1):
				next = offsetLp[i+1]
			else:
				next = offsetLp[0]		
						
			if type(e[0]) == dxfgrabber.dxfentities.Line:
				if type(prev[0]) == dxfgrabber.dxfentities.Line: 
				#convert lines to y = mx+c form from P1 to P2 form										
					x1, y1 = self.intersectLines(e,prev)			
				else:	
					if prev[1]:					
						x1,y1 = self.startAndEndCartesian(prev[0])[1]
					
					else:
						x1,y1 = self.startAndEndCartesian(prev[0])[0]

				if type(next[0]) == dxfgrabber.dxfentities.Line: 
					x2,y2 = self.intersectLines(e,next)
				else:	
					if next[1]:					
						x2, y2 = self.startAndEndCartesian(next[0])[0]						
					else:
						x2,y2 = self.startAndEndCartesian(next[0])[1]				
				e[0].start = (x1 , y1)
				e[0].end = (x2, y2)
				offsetLp[i] = (e[0] , True ,e[2])
		return offsetLp
	
	def drawDXFElementOnImage(self,e,img,n,pxpermm,xoffset,yoffset):
		font                   = cv2.FONT_HERSHEY_SIMPLEX
		fontScale              = 1
		fontColor              = (255,255,255)
		lineType               = 2
		colour = (0,255,0)
		G2 = (255,255,0)
		G3 = (0,255,255)
		thickness = 2

		pts = self.startAndEndCartesian(e[0])
		if type(e[0]) == dxfgrabber.dxfentities.Arc:
			if e[1]:
				col = G2
			else:
				col = G3				
			angle = 0
			startAngle = int(e[0].start_angle)
			endAngle = int(e[0].end_angle)
			size = (int(e[0].radius* pxpermm),int(e[0].radius* pxpermm))
			centre = (int(e[0].center[0]*pxpermm+xoffset),int(e[0].center[1]*pxpermm)+yoffset)
			cv2.ellipse(img, centre ,size, angle, startAngle, endAngle,color= col,thickness= 5) 			
			if n != None: cv2.putText(img,str(n), centre, font, fontScale,fontColor,lineType)									
			if self.Xmax < (e[0].center[0]+e[0].radius): self.Xmax = e[0].center[0]+e[0].radius
			if self.Xmin > (e[0].center[0]-e[0].radius): self.Xmin = e[0].center[0]-e[0].radius		
		elif type(e[0]) == dxfgrabber.dxfentities.Line:
			x1 = int(e[0].start[0] * pxpermm)
			y1 = int(e[0].start[1] * pxpermm)
			x2 = int(e[0].end[0] * pxpermm)
			y2 = int(e[0].end[1] * pxpermm)
			if n != None: cv2.putText(img,str(n), (int(x1 + (x2-x1)/2 + xoffset),int(y1 + (y2-y1)/2 + yoffset)), font, fontScale,fontColor,lineType)
			cv2.line(img, (x1+xoffset, y1+yoffset), (x2+xoffset, y2+yoffset), colour, thickness)
			if self.Xmin > e[0].start[0]: self.Xmin = e[0].start[0]
			if self.Xmin > e[0].end[0]: self.Xmin = e[0].end[0]
			if self.Xmax < e[0].start[0]: self.Xmax = e[0].start[0]
			if self.Xmax < e[0].end[0]: self.Xmax = e[0].end[0]
			else:
				print("Unsported entitie type: ", type(e[0]))
				print("norm:", e[2][0]* pxpermm , ",", e[2][1]* pxpermm)											
				if e[1]:
					x1 = int(pts[0][0] * pxpermm)
					y1 = int(pts[0][1] * pxpermm)
					x2 = int(x1 + e[2][0] * pxpermm)
					y2 = int(y1 + e[2][1] * pxpermm)
				else:
					x1 = int(pts[1][0] * pxpermm)
					y1 = int(pts[1][1] * pxpermm)
					x2 = int(x1 + e[2][0] * pxpermm)
					y2 = int(y1 + e[2][1] * pxpermm)
				if n != None: cv2.line(img, (x1+xoffset, y1+yoffset), (x2+xoffset, y2+yoffset), (150,150,150), thickness)
				cv2.circle(img,(x1+xoffset,y1+yoffset), 5, (0,0,255), -1)

	def visualise(self,pxpermm,lp,offset):
		pxpermm  = float(pxpermm)
		img = np.zeros((1000,1000,3), np.uint8)
		xoffset = 100
		yoffset = 100		
		n = 0
		loop = self.LoopNormals(lp,offset)
		for e in loop:
			n += 1
			self.drawDXFElementOnImage(e,img,n,pxpermm,xoffset,yoffset)
		offsetLp = self.offsetLoop(loop[:])			
		for e in loop:
			self.drawDXFElementOnImage(e,img,None,pxpermm,xoffset,yoffset)
		self.loop = offsetLp		
		return img				

	def toGcode(self):
		gscale = 1
		Gcode = "G21 G90 F100\n"
		first = self.startAndEndCartesian(self.loop[0][0])		
		if self.loop[0][1]:		
			Gcode += "G00 X" + str(round(first[0][0]*gscale,3)) + " Y" + str(round(first[0][1]*gscale,3)) + "\n"
		else:
			Gcode += "G00 X" + str(round(first[1][0]*gscale,3)) + " Y" + str(round(first[1][1]*gscale,3)) + "\n" 
		
		Gcode += "G01 Z-2\n"
		for e in self.loop:
			first = self.startAndEndCartesian(e[0])
			if type(e[0]) == dxfgrabber.dxfentities.Arc:
				if e[1]:
					print("c1: " + str(round(first[0][0]*gscale,3)) + "," + str(round(first[0][1]*gscale,3)) +  " c2: " + str(round(first[1][0]*gscale,3)) + "," + str(round(first[1][1]*gscale,3)))
					Gcode +=  "G03 X"  + str(round(first[1][0]*gscale,3))  + " Y" + str(round(first[1][1]*gscale,3)) + " R" + str(round(e[0].radius*gscale,3)) + "\n" 
				else:
					Gcode += "G02 X"  + str(round(first[0][0]*gscale,3))  + "  Y" + str(round(first[0][1]*gscale,3)) + " R" + str(round(e[0].radius*gscale,3)) + "\n" 
			if type(e[0]) == dxfgrabber.dxfentities.Line:
				if e[1]:
					Gcode += "G01 X" + str(round(first[1][0]*gscale,3)) + " Y" + str(round(first[1][1]*gscale,3)) + "\n"
				else:
					Gcode += "G01 X" + str(round(first[0][0]*gscale,3)) + " Y" + str(round(first[0][1]*gscale,3)) + "\n"
		return Gcode

ptspermm = 100
#p = profile("profile.dxf")
	
os.system("cp ~/dxf/profile.dxf profile.dxf") 
p = profile("profile.dxf")	
p2 = profile("profile.dxf")	

v = p.visualise(ptspermm,p.findDxfLoop(),0.4)
v2 = p2.visualise(ptspermm,p2.findDxfLoop(),-0.4)

cv2.imwrite("a.png", v)	
cv2.imwrite("b.png", v2)	
l1 = p.loopLen()
l2 = p2.loopLen()

if l1 > l2:
	cv2.imwrite("c.png", v)
	open("gout.nc","w").write(p.toGcode())	
else:
	cv2.imwrite("c.png", v2)	
	p2.toGcode()
	open("gout.nc","w").write(p1.toGcode())


