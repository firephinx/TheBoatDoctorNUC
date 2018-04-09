import numpy as np 
import cv2  
import copy





class rgb_process(object):
	def __init__(self,img):
		self.wide_range=1;  ### how much colns to save   0.3
		self.left_coln=np.true_divide(1-self.wide_range,2); ### empty number of left and right colns

		self.wide_range_corner=0.3;  ### how much colns to save 
		self.left_coln_corner=np.true_divide(1-self.wide_range_corner,2); ### empty number of left and right colns
		self.offset=0.2
		self.img=img
		#print 1



	def mask(self,img,station_F):  
	###  make mask on image, so that only shows middle potion of img
	###  if at station F, then only show left portion of img becasue Kinnect camera is at right side 
	###   input: img= RGB img;  station_F= if the task is at station_F

		try:
			size=np.shape(img)
			row=size[0]
			coln=size[1]
			#print row,coln
			mask=np.zeros([row,coln],np.uint8)
			#print np.shape(mask)
			#print np.shape(mask[:,0])
			
			if station_F==0:
				for i in np.arange(int(self.left_coln*coln),int((self.left_coln+self.wide_range)*coln)):
					mask[:,i]=255
			elif station_F ==1:
				for i in np.arange(int((self.left_coln-self.offset)*coln),int((self.left_coln+self.wide_range_corner-self.offset)*coln)):
					mask[:,i]=255

			masked_img=cv2.bitwise_and(img,img,mask=mask)
			###### helper function ##########
			# cv2.imshow('result',masked_img)
			# cv2.waitKey(0)
			##### ends ##############
			return masked_img
		except:
			print "[rgb_processing] mask funciton Error"


		

	def returnBound(self,binary_img,shape,img):
		### return bound from binary_image through contour 
		### input: cnts= contours;  shape= "circle" or "square", img = for drawing purpose
		_,cnts,_=cv2.findContours(binary_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		for cnt in cnts: 
			if shape=="square":
				rect = cv2.minAreaRect(cnt)
				bound = cv2.boxPoints(rect)
				bound = np.int0(bound)
			#### helper lines: to see if contours are coorectly found #### 
				cv2.drawContours(img,[bound],0,(0,255,0),2)
			elif shape=="circle":
				bound = cv2.fitEllipse(cnt)
			###### helper lines: to see ....##########
				cv2.ellipse(img,bound,(0,255,0),2)
		#cv2.imshow('result4',img)
		#cv2.waitKey(0)	
		return bound


	def findTarget(self,binary_img,th):
		###### this function returns a mask that contains targets
		###### input: binary_img=binary_img 
		######      : th= area threshold  
		#try:
			_, labels, stats, _= cv2.connectedComponentsWithStats(binary_img)

			goodLabels=[label for label in np.arange(np.max(labels)+1) if stats[label,4]>th and label!=0]
			#print goodLabels
			#print stats[1,4]
			#cv2.imshow("a",binary_img)
			#cv2.waitKey(0)
			label_length=np.size(goodLabels)
			#print label_length
			if label_length==0:
				#print "Im here"
				#print np.shape(binary_img)
				final_mask= None
			elif label_length==1:
				#print "Im here2"
				final_mask=(labels>=goodLabels[0])
			elif label_length>1:
				#print "Im here3"
				final_label=(labels==goodLabels[0])
				for i in np.arange(1,label_length):	
					final_label=(final_label | (labels==goodLabels[i])) ## find pixel position that satisfy the area requirement from goodLabels
					#print final_label
				final_mask=final_label
				#print ("im here")
			#print ((labels>1) & (labels <3))
			####### helper line: to check if right mask)
			#masked_img=cv2.bitwise_and(binary_img,binary_img,mask=final_mask)
			#cv2.imshow("result3",masked_img)
			#cv2.waitKey(0)
			#print goodLabels
			#print stats
			#print np.max(labels)
			return final_mask
		#except:
		#	print "[rgb_processing] findTarget funciton error"
	#		return None


	def th_hsv(self,img,low_th,high_th):
		### this function converts to hsv and return binary mask after thresholding 
		#### img: RGB img 
		#### 
		#print 1
		img_hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
		#print 2
		##### helper line ######
		cv2.imshow("result1",img_hsv)
		cv2.waitKey(20)
		#########################
		mask=cv2.inRange(img_hsv,low_th,high_th) 
		#cv2.imshow("result2",mask)
		#cv2.waitKey(200)
		#print 3
		##### helper line ########
		img=cv2.bitwise_and(img_hsv,img_hsv,mask=mask)
		cv2.imshow("result2",img)
		cv2.waitKey(20)
		########################
		return mask 




	def th_color(self,img,low_th,high_th):
		### this function converts to hsv and return binary mask after thresholding 
		#### img: RGB img 
		#### 
		#print 1
		#img_hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
		#print 2
		mask=cv2.inRange(img,low_th,high_th) 
		#cv2.imshow("result2",mask)
		#cv2.waitKey(200)
		#print 3
		##### helper line ########
		# img=cv2.bitwise_and(img_hsv,img_hsv,mask=mask)
		# cv2.imshow("result2",img)
		# cv2.waitKey(0)
		########################
		return mask 


	 

class kinect_process(rgb_process):
	def __init__(self,img):
		self.wide_range=0.2;  ### how much colns to save   0.3
		self.left_coln=np.true_divide(1-self.wide_range,2); ### empty number of left and right colns

		self.wide_range_corner=0.3;  ### how much colns to save 
		self.left_coln_corner=np.true_divide(1-self.wide_range_corner,2); ### empty number of left and right colns
		self.offset=0.2
		self.img=img


########## proc each acutaor ###############

	def breaker_proc(self,img,area_th):
		######## the function process breaker 
		####### input: RGB image 
		#cv2.imshow("result4",img)
		#cv2.waitKey(0)
		low_th=np.array([0,170,100])
		high_th=np.array([20,200,255])
		mask=self.th_hsv(img,low_th,high_th)
		target_mask=self.findTarget(mask,area_th)
		##### helper line ###############
		# mask_final=np.uint8(target_mask)
		# masked_img=cv2.bitwise_and(img,img,mask=mask_final)
		# cv2.imshow("result4",masked_img)
		# cv2.waitKey(0)
		###############################
		if target_mask is None:
			size=np.shape(img)
			row=size[0]
			coln=size[1]
			target_mask=np.zeros([row,coln],np.uint8)
			#cv2.imshow("mask",target_mask)
	  		#cv2.waitKey(200)
	  	#cv2.imshow("mask",target_mask)
	  	#cv2.waitKey(200)	
	  	return target_mask


	def vertical_proc(self,img,area_th):
			######## the function process breaker 
			####### input: RGB image 
			#cv2.imshow("result4",img)
			#cv2.waitKey(0)
			#low_th=np.array([100,120,120])
			low_th=np.array([70,110,100])
			high_th=np.array([255,255,255])
			mask=self.th_hsv(img,low_th,high_th)
			target_mask=self.findTarget(mask,area_th)
			##### helper line ###############
			# mask_final=np.uint8(target_mask)
			# masked_img=cv2.bitwise_and(img,img,mask=mask_final)
			# cv2.imshow("result4",masked_img)
			# cv2.waitKey(0)
			###############################
			if target_mask is None:
				size=np.shape(img)
				row=size[0]
				coln=size[1]
				target_mask=np.zeros([row,coln],np.uint8)
				#cv2.imshow("mask",target_mask)
		  		#cv2.waitKey(200)
		  	#cv2.imshow("mask",target_mask)
		  	#cv2.waitKey(200)	
		  	return target_mask




	def small_blue(self,img,area_th):
		low_th=np.array([0,0,0])
		high_th=np.array([255,255,255])
		mask=self.th_hsv(img,low_th,high_th)
		target_mask=self.findTarget(mask,area_th)

	def orange(self,img,area_th):
		low_th=np.array([0,0,0])
		high_th=np.array([255,255,255])
		mask=self.th_hsv(img,low_th,high_th)
		mask_final=np.uint8(mask)
		target_mask=self.findTarget(mask,area_th)
		##### helper line ###########
		# mask_final=np.uint8(target_mask)
		# masked_img=cv2.bitwise_and(img,img,mask=mask_final)
		# cv2.imshow("result4",masked_img)
		# cv2.waitKey(0)
		#############################


	def L_shape(self,img,area_th):
		low_th=np.array([100,100,100])
		high_th=np.array([255,255,255])
		mask=self.th_hsv(img,low_th,high_th)
		target_mask=self.findTarget(mask,area_th)



	def horizontal_blue(self,img,area_th):
		img2=copy.deepcopy(img)
		
		low_th=np.array([100,120,150])
		high_th=np.array([255,255,255])
		mask1=self.th_hsv(img,low_th,high_th)
		

		# low_th=np.array([50,0,100])
		# high_th=np.array([150,30,255])
		# mask2=self.th_hsv(img2,low_th,high_th)


		# low_th=np.array([0,0,100])
		# high_th=np.array([50,30,255])
		# mask3=self.th_hsv(img2,low_th,high_th)

		# mask_final=((mask1|mask2))
		mask_final=mask1

		# masked_img=cv2.bitwise_and(img,img,mask=mask_final)
		# cv2.imshow("result3",masked_img)
		# cv2.waitKey(0)

		target_mask2=self.findTarget(mask_final,400)
		_,binary=cv2.threshold(mask_final,100,255,cv2.THRESH_BINARY);
		########## helper line ##########################
		#masked_img=cv2.bitwise_and(img,img,mask=binary)
		cv2.imshow("result3",binary)
		cv2.waitKey(0)
		#################################################



	def locate_actuators(self,type,station_F,img):
	#### process different type of actuato
		#### input: img: RGB img
		#### type: actutator type  1: B , 2: vertial 3: small 4: orange 5: L_shape 6: A  7:horizontal
		##### station_F: if at station F 
		#print 1 
		masked_img=self.mask(img,station_F)
		#print np.shape(img)
		if type==1:
			blob_area_th=200
			target_mask=self.breaker_proc(masked_img,blob_area_th)
		elif type==2:
			blob_area_th=10
			target_mask=self.vertical_proc(masked_img,blob_area_th)
		elif type==3: ### haven't tested yet 
			blob_area_th=400
			self.samll_blue(masked_img,blob_area_th)
		elif type==4: ### haven't tested yet 
			blob_area_th=400
			self.orange(masked_img,blob_area_th)
		elif type==5:
			blob_area_th=400
			self.L_shape(masked_img,blob_area_th)
		elif type==6: ### same as B for right now 
			blob_area_th=400
			self.breaker_proc(masked_img,blob_area_th)
		elif type==7: 
			blob_area_th=400
			self.horizontal_blue(masked_img,blob_area_th)
		return target_mask




class pi_cam_process(rgb_process):
	def __init__(self,img):
		self.wide_range=1;  ### how much colns to save   0.3
		self.left_coln=np.true_divide(1-self.wide_range,2); ### empty number of left and right colns

		self.wide_range_corner=0.3;  ### how much colns to save 
		self.left_coln_corner=np.true_divide(1-self.wide_range_corner,2); ### empty number of left and right colns
		self.offset=0.2
		self.img=img


	def green_tip_orange(self,img,area_th):
			######## the function process breaker 
			####### input: RGB image 
			#cv2.imshow("result4",img)
			#cv2.waitKey(0)
			low_th=np.array([20,80,20])
			high_th=np.array([100,100,100])
			mask=self.th_hsv(img,low_th,high_th)
			target_mask=self.findTarget(mask,area_th)
			if target_mask is None:
				size=np.shape(img)
				row=size[0]
				coln=size[1]
				target_mask=np.zeros([row,coln],np.uint8)
				#cv2.imshow("mask",target_mask)
		  		#cv2.waitKey(200)
		  	
			##### helper line ###############
			# mask_final=np.uint8(target_mask)
			# masked_img=cv2.bitwise_and(img,img,mask=mask_final)
			# cv2.imshow("result4",masked_img)
			# cv2.waitKey(0)
			###############################
			


		  	#cv2.imshow("mask",target_mask)
		  	#cv2.waitKey(200)	
		  	return target_mask



	def green_tip_small(self,img,area_th):
			######## the function process breaker 
			####### input: RGB image 
			#cv2.imshow("result4",img)
			#cv2.waitKey(0)
			low_th=np.array([20,50,20])
			high_th=np.array([100,120,50])
			mask=self.th_hsv(img,low_th,high_th)
			target_mask=self.findTarget(mask,area_th)
			if target_mask is None:
				size=np.shape(img)
				row=size[0]
				coln=size[1]
				target_mask=np.zeros([row,coln],np.uint8)
				#cv2.imshow("mask",target_mask)
		  		#cv2.waitKey(200)
		  	
			##### helper line ###############
			# mask_final=np.uint8(target_mask)
			# masked_img=cv2.bitwise_and(img,img,mask=mask_final)
			# cv2.imshow("result4",masked_img)
			# cv2.waitKey(0)
			###############################
			


		  	#cv2.imshow("mask",target_mask)
		  	#cv2.waitKey(200)	
		  	return target_mask	  	



	def orange_proc(self,img,area_th):
			######## the function process breaker 
			####### input: RGB image 
			#cv2.imshow("result4",img)
			#cv2.waitKey(0)
			low_th=np.array([0,100,40])
			high_th=np.array([100,200,100])
			mask=self.th_hsv(img,low_th,high_th)
			target_mask=self.findTarget(mask,area_th)
			
			if target_mask is None:
				size=np.shape(img)
				row=size[0]
				coln=size[1]
				target_mask=np.zeros([row,coln],np.uint8)
				#cv2.imshow("mask",target_mask)
		  		#cv2.waitKey(200)
		  	##### helper line ###############
		  	#print 1
			#mask_final=np.uint8(target_mask)
			#masked_img=cv2.bitwise_and(img,img,mask=mask_final)
			#cv2.imshow("masked_img",masked_img)
			#cv2.waitKey(0)
			###############################
		  	return target_mask


	def small_proc(self,img,area_th):
			######## the function process small valve 
			####### input: RGB image 
			#cv2.imshow("result4",img)
			#cv2.waitKey(0)
			low_th=np.array([100,120,0])
			high_th=np.array([255,255,100])
			mask=self.th_hsv(img,low_th,high_th)
			target_mask=self.findTarget(mask,area_th)
			
			if target_mask is None:
				size=np.shape(img)
				row=size[0]
				coln=size[1]
				target_mask=np.zeros([row,coln],np.uint8)
				#cv2.imshow("mask",target_mask)
		  		#cv2.waitKey(200)
		  	##### helper line ###############
		  	#print 1
			# mask_final=np.uint8(target_mask)
			# masked_img=cv2.bitwise_and(img,img,mask=mask_final)
			# cv2.imshow("masked_img",masked_img)
			# cv2.waitKey(0)
			###############################
		  	return target_mask


	def switch_proc(self,img,area_th):
			######## the function process small valve 
			####### input: RGB image 
			#cv2.imshow("result4",img)
			#cv2.waitKey(0)
			low_th=np.array([100,120,0])
			high_th=np.array([255,255,100])
			mask=self.th_hsv(img,low_th,high_th)
			target_mask=self.findTarget(mask,area_th)
			
			if target_mask is None:
				size=np.shape(img)
				row=size[0]
				coln=size[1]
				target_mask=np.zeros([row,coln],np.uint8)
				#cv2.imshow("mask",target_mask)
		  		#cv2.waitKey(200)
		  	##### helper line ###############
		  	#print 1
			# mask_final=np.uint8(target_mask)
			# masked_img=cv2.bitwise_and(img,img,mask=mask_final)
			# cv2.imshow("masked_img",masked_img)
			# cv2.waitKey(0)
			###############################
		  	return target_mask




	def find_circle_center(self,cnt,img):
		##### ths function find center of a bounding circle ######
		### input: cnt: one contour 
		#### img: orignal rgb img for drawing 
		#### return: center of a bounding circle
			(x,y),radius = cv2.minEnclosingCircle(cnt)
			##### helper funciton #####
			# center_int = (int(x),int(y))	
			# radius_int = int(radius)
			# cv2.circle(img,center_int,radius_int,(0,255,0),2)
			# cv2.imshow("circle",img)
			# cv2.waitKey(0)
			#### ends #########
			return (x,y)



	def find_rect_center(self,cnt,img):
		##### ths function find center of a bounding box ######
		### input: cnt: one contour 
		#### img: orignal rgb img for drawing 
		#### return: center of a bounding box  
			rect = cv2.minAreaRect(cnt)
			box = cv2.boxPoints(rect)
			Cx=np.average(box[:,0])  ### find X center of box 
			Cy=np.average(box[:,1])  ### fiind Y center of box 
			#### helper function ####
			# int_box = np.int0(box)
			# cv2.drawContours(img,[int_box],0,(0,0,255),2)
			# cv2.imshow("rect",img)
			# cv2.waitKey(0)
			#### ends ####

			return (Cx,Cy)


	def find_line(self,cnt,img):
		##### ths function find two poitns that fit a line of a contour ######
		### input: cnt: one contour 
		#### img: orignal rgb img for drawing 
		#### return: direction vector (vx,vy) and a poit on the line (x,y)
		vx,vy,x,y = cv2.fitLine(cnt, cv2.DIST_L2,0,0.01,0.01)
		#lefty = int((-x*vy/vx) + y)
		#righty = int(((cols-x)*vy/vx)+y)
		### helper function ####
		# print vx,vy,x,y
		# cv2.line(img,(int(x-vx*500),int(y-vy*500)),(int(x+vx*100),int(y+vy*100)),(0,255,0),2)
		# cv2.imshow("fitted_line",img)
		# cv2.waitKey(0)
		##### ends #########
		return (vx,vy,x,y)	



	def unroated_box(self,cnt,img):
		###### this function find cebter of an unroated bounding box ########
		### input: cnt: one contour 
		#### img: orignal rgb img for drawing 
		#### return: center of a bounding box  
			x,y,w,h = cv2.boundingRect(cnt)
			#### helper function ####
			# cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
			# cv2.imshow("unroated_rect",img)
			# cv2.waitKey(0)
			#### ends ####
			return (x,y,w,h)
		

	def find_contour_thresh(self,mask,area_th,img):
	#### function: this function find contour in a mask that has largest area  
	#### mask: 0,1 binary img 
	#### area_th: area threshhold for contour 
	#### img: RGB img for drawing 
	#### return: filtered contours  
			mask=np.uint8(mask)
			_,contours,_= cv2.findContours(mask, 1, 2)
			#cnts= [cnt for cnt in contours if cv2.contourArea(cnt)>area_th]
			length=len(contours)
			if length==0:  ### if no contour find 
				return None
			else:  
				area_max=cv2.contourArea(contours[0])
				cnt=contours[0]
				if length>1:
					i=0
					for cnts in contours: 
						area2=cv2.contourArea(contours[i])
						if area2>area_max: 
							area_max=area2
							cnt=contours[i]
						i+=1
				###### helper function ######
				# cv2.drawContours(img, cnt, -1, (0,255,0), 3)
				# cv2.imshow("contours",img)
				# cv2.waitKey(0)
				###### helper function ends ######
				
				if area_max>area_th:
					return cnt 
				else:
					return None 
	
		  	

	def calc_angle_circle(self,center,pt,img):
	#### this function calcualte anlge of line respect to +x axis in cw 
	#### input: center, pt2: two pts in tuple; center is the center of a circle 
	#### return anlge with respect to +x in cw  
			vec=np.array(pt)-np.array(center)
			angle=np.angle(vec[0]+vec[1]*1j,deg=True)
		##### helper function #####
			# center_int=(int(center[0]),int(center[1]))
			# pt_int=(int(pt[0]),int(pt[1]))
			# cv2.line(img, center_int, pt_int, (255,0,0), 2)
			#print angle
			# cv2.imshow("Line",img)
			# cv2.waitKey(0) 
		####### ends ##########
			return angle 

	def get_ROI(self,ROI_coord,img):
	#### This function get ROI for green tip 
	#### input: ROT_coord: (x,y,w,h) for ROI 
	#### img: orignal RGB img 
	#### return: masked ROI img 
			size=np.shape(img)
			row=size[0]
			coln=size[1]
			#print row,coln
			mask=np.zeros([row,coln],np.uint8)
			#print np.shape(mask)
			#print np.shape(mask[:,0])
			x=ROI_coord[0]
			y=ROI_coord[1]
			w=ROI_coord[2]
			h=ROI_coord[3]
			mask[y:y+h,x:x+w]=255
	
			masked_img=cv2.bitwise_and(img,img,mask=mask)
			
			#### helper function ####
			# cv2.imshow('ROI',masked_img)
			# cv2.waitKey(0)
			#### ends ######
			return masked_img




	########## breaker processs ################

	def breaker_proc(self,img,area_th):
			######## the function process breaker 
			####### input: RGB image 
			#cv2.imshow("result4",img)
			#cv2.waitKey(0)
			low_th=np.array([0,180,0])
			high_th=np.array([100,255,80])
			mask=self.th_hsv(img,low_th,high_th)
			target_mask=self.findTarget(mask,area_th)
			
			if target_mask is None:
				size=np.shape(img)
				row=size[0]
				coln=size[1]
				target_mask=np.zeros([row,coln],np.uint8)
				#cv2.imshow("mask",target_mask)
		  		#cv2.waitKey(200)
		  	##### helper line ###############
		  	#print 1
			# mask_final=np.uint8(target_mask)
			# masked_img=cv2.bitwise_and(img,img,mask=mask_final)
			# cv2.imshow("masked_img",masked_img)
			# cv2.waitKey(0)
			###############################
			return target_mask


	def find_three_contours_thresh(self,mask,area_th,img,breaker):
		#### function: this function find contour in a mask that has largest area  
		#### mask: 0,1 binary img 
		#### area_th: area threshhold for contour 
		#### img: RGB img for drawing 
		#### return: filtered contours  
			mask=np.uint8(mask)
			_,contours,_= cv2.findContours(mask, 1, 2)
			#cnts= [cnt for cnt in contours if cv2.contourArea(cnt)>area_th]
			length=len(contours)
			if length==0:  ### if no contour find 
				return None
			else:  
				if length>=3:
					cnts=[]
					i=0 
					cnt_area=dict()
					for cnt in contours:
						cnts.append(cnt)
						cnt_area[str(i)]=cv2.contourArea(cnt)
						i+=1
					#print cnt_area
					#print sorted(cnt_area,key=cnt_area.get)
					ordered_key=sorted(cnt_area,key=cnt_area.get)
					chosen_key=ordered_key[-3:len(ordered_key)]  ### get three contours that have largest area 
					cnts_chosen=[] 
					for key in chosen_key:
						if cnt_area[key]>area_th:
							cnts_chosen.append(cnts[int(key)])
					#print chosen_key 
					###### helper function ######
					if len(cnts_chosen)<3:
						if breaker==1:
							print "[pi_cam_info]: I only find {} brekers".format(len(cnts_chosen))
						else: 
							print "[pi_cam_info]: I only find {} black_box".format(len(cnts_chosen)) 
						return None 
					else:
						if breaker==1:
							print "[pi_cam_info]: I find 3 brekers"
						else: 
							print "[pi_cam_info]: I find 3 black_box" 
						return cnts_chosen
					##### helper function ends ######
					# cv2.drawContours(img, cnts_chosen, -1, (0,255,0), 3)
					# cv2.imshow("contours"+str(breaker),img)
					# cv2.waitKey(0)
					###### helper function ends ######
				else:
					if breaker==1:
						print "[pi_cam_info]: I only find {} brekers".format(length)
					else:
						print "[pi_cam_info]: I only find {} black_box".format(length)
					return None  
				
	def find_multi_rect_centers(self,cnts,img,breaker):
		##### ths function find center of a bounding box ######
		### input: cnt: one contour 
		#### img: orignal rgb img for drawing 
		#### return: breaker_lsit that contains each breaker's dictionary info (3 in total), the dictionary includes Center X, center Y, and four vertices info
			breaker_list=[]

			for cnt in cnts:   
				rect = cv2.minAreaRect(cnt)
				box = cv2.boxPoints(rect)
				Cx=np.average(box[:,0])  ### find X center of box 
				Cy=np.average(box[:,1])  ### fiind Y center of box
				breaker_info=dict()
				breaker_info["Cx"]=Cx
				breaker_info["Cy"]=Cx
				breaker_info["box"]=box 
				breaker_list.append(breaker_info)
				 
			#### helper function ####
			# 	int_box = np.int0(box)
			# 	cv2.drawContours(img,[int_box],0,(0,0,255),2)
			# cv2.imshow("rect"+str(breaker),img)
			# cv2.waitKey(0)
			#### ends ####
			breaker_list=sorted(breaker_list, key=lambda k: k["Cx"]) ### sort dictionary so that from left to right
			return breaker_list


	def get_blackBox_ROI(self,breaker_info_s,img):
		##### this function find ROI of black box around breakers######
		##### input: breaker_info 
		#### img: orignal rgb img 
		#### return: maksed img that contains only black box and breakers 
		size=np.shape(img)
		row=size[0]
		coln=size[1]
		#print row,coln
		mask=np.zeros([row,coln],np.uint8)
		scale_width=2; ## scale factor of breaker box to include black box 
		scale_heigth=6; ## scale factor of breaker box to include black box 

		for breaker_info in breaker_info_s:
			box=breaker_info['box']
			x_min=np.min(box[:,0])
			y_min=np.min(box[:,1])
			breaker_width=np.max(box[:,0])-x_min
			breaker_height=np.max(box[:,1])-y_min
			#print breaker_width,breaker_height
			x_start=int(x_min-breaker_width)
			x_end=int(x_min+breaker_width*scale_width)
			y_start=int(y_min-breaker_height*scale_heigth)
			y_end=int(y_min+breaker_height*scale_heigth)
			mask[y_start:y_end+1,x_start:x_end+1]=255
		####### helper function ######### 
		masked_img=cv2.bitwise_and(img,img,mask=mask)
		###### helper function ##########
		# cv2.imshow('black_box',masked_img)
		# cv2.waitKey(0)
		##### ends ##############
		return masked_img		


	def black_box_proc(self,img,area_th):
		######## the function process breaker 
		####### input: RGB image 
		#cv2.imshow("result4",img)
		#cv2.waitKey(0)
		low_th=np.array([30,45,0])
		high_th=np.array([60,60,20])
		##low_th=np.array([10,10,10])
		#high_th=np.array([15,15,15])
		mask=self.th_hsv(img,low_th,high_th)

		#cv2.imshow("masked_img_black_box2",mask)
		#cv2.waitKey(0)

		target_mask=self.findTarget(mask,area_th)
		
		if target_mask is None:
			size=np.shape(img)
			row=size[0]
			coln=size[1]
			target_mask=np.zeros([row,coln],np.uint8)
			#cv2.imshow("mask",target_mask)
	  		#cv2.waitKey(200)
	  	##### helper line ###############
	  	#print 1
	  	#print 1
		# mask_final=np.uint8(target_mask)

		# cv2.imshow("masked_img_black_box2q",img)
		# cv2.waitKey(0)
		
		# masked_img=cv2.bitwise_and(img,img,mask=mask_final)
		# cv2.imshow("masked_img_black_box",masked_img)
		# cv2.waitKey(0)
		###############################
		return target_mask

	

	def breaker_status(self,breakers_info,black_boxes_info):
		######## this function determines if a beaker is on or off 
		######## input: breakers_info: dictionanry of berakers_info from left to right 
		#######  black_boxes_info: dictionanry of black_boxes_info from left to right
		####### return: breaker state: up or down
		states=[] 
		for i in np.arange(3):
			breaker_Cy=breakers_info[i]["Cy"]
			box_Cy=black_boxes_info[i]["Cy"]
			if breaker_Cy<=box_Cy*1.1: 
				state="Down"
			else:
				state="Up"
			states.append(state)
		return states	



	######### ends ######################


	def locate_green(self,type,img,station_F=0):
			masked_img=self.mask(img,station_F)
			img_visual=copy.deepcopy(img)
			if type==3: ## orange valve

				####### find center of orange valve ########
				orange_area_th=50000
				orange_mask=self.orange_proc(masked_img,orange_area_th)
				cnt_area_th=50000
				cnt=self.find_contour_thresh(orange_mask,cnt_area_th,img_visual)
				if cnt is None: 
					print "[pi_cam_info]: I can't find small valve" 
					return None 
				else:
					circle_center=self.find_circle_center(cnt,img_visual)
					x,y,w,h=self.unroated_box(cnt,img_visual)
					masked_img=self.get_ROI((x,y,w,h),img_visual)

				######## find center of green tip #########
					tip_area_th=400
					tip_mask=self.green_tip_orange(masked_img,tip_area_th)
					cnt_area_th=400
					cnts= self.find_contour_thresh(tip_mask,cnt_area_th,img_visual)
					if cnts is None: 
						print "[pi_cam_info]: I can't find green tip on small valve" 
						return None 
					else:					
						tips_center=self.find_rect_center(cnts[0],img_visual)	
						angle=self.calc_angle_circle(circle_center,tips_center,img_visual)
						print "[pi_cam_info] Orange valve orientation is",angle,"deg"

			elif type==4: ## small valve 
				small_area_th=10000
				small_mask=self.small_proc(masked_img,small_area_th)
				cnt_area_th=10000
				cnt= self.find_contour_thresh(small_mask,cnt_area_th,img_visual)
				if cnt is None: 
					print "[pi_cam_info]: I can't find small blue valve" 
					return None 
				else: 
					circle_center=self.find_circle_center(cnt,img_visual)
					x,y,w,h=self.unroated_box(cnt,img_visual)
					masked_img=self.get_ROI((x,y,w,h),img_visual)

				#print tips_center  
					tip_area_th=400
					tip_mask=self.green_tip_small(masked_img,tip_area_th)
					cnt_area_th=400
					cnt= self.find_contour_thresh(tip_mask,cnt_area_th,img_visual)
					if cnt is None: 
						print "[pi_cam_info]: I can't find green tip on small valve" 
						return None 
					else:
						tips_center=self.find_rect_center(cnt,img_visual)
						angle=self.calc_angle_circle(circle_center,tips_center,img_visual)
						print "[pi_cam_info] Small blue valve orientation is",angle,"deg"

			elif type==2: ## small switch 
				small_area_th=10000
				small_mask=self.switch_proc(masked_img,small_area_th)
				cnt_area_th=10000
				cnt= self.find_contour_thresh(small_mask,cnt_area_th,img_visual)
				if cnt is None: 
					print "[pi_cam_info]: I can't find horizontal blue switch" 
					return None 
				else: 
					vx,vy,_,_=self.find_line(cnt,img_visual)
					angle=np.angle(vx+vy*1j,deg=True)
					print "The swithc is ", angle, "deg" 
					#x,y,w,h=self.unroated_box(cnt,img_visual)
					#masked_img=self.get_ROI((x,y,w,h),img_visual)

			elif type==1: ## 
				breaker_area_th=500
				breaker_mask=self.breaker_proc(masked_img,breaker_area_th)
				cnt_area_th=500
				cnts=self.find_three_contours_thresh(breaker_mask,cnt_area_th,img_visual,breaker=1)
				if cnts is None: 
					return None
				else: 
					breaker_info=self.find_multi_rect_centers(cnts,img_visual,breaker=1)
					masked_image=self.get_blackBox_ROI(breaker_info,img)
					black_box_aera_th=500
					black_box_mask=self.black_box_proc(masked_image,black_box_aera_th)
					cnt_area_th=500
					cnts=self.find_three_contours_thresh(black_box_mask,cnt_area_th,img_visual,breaker=0)
					if cnts is None: 
						return None
					else :
						black_box_info=self.find_multi_rect_centers(cnts,img_visual,breaker=0)
						breaker_states=self.breaker_status(breaker_info,black_box_info)
						print breaker_states
						#print black_box_info 




			







if __name__=="__main__":
	path= "../opencv_image/pi_cam_image/1.png"
	img=cv2.imread(path)
	#cv2.imshow('result0',img)
	#cv2.waitKey(0)
	

	# kinect=kinect_process(img)
	# Type=2
	# station_F=0
	# final_mask=kinect.locate_actuators(Type,station_F,img)


	PI=pi_cam_process(img)
	Type=1
	final_mask=PI.locate_green(Type,img)