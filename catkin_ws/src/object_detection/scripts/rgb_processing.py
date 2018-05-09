import numpy as np 
import cv2  
import copy

import params 




class rgb_process(object):
	def __init__(self,img):
		self.wide_range=1;  ### how much colns to save   0.3
		self.left_coln=np.true_divide(1-self.wide_range,2); ### empty number of left and right colns

		self.wide_range_corner=0.3;  ### how much colns to save 
		self.left_coln_corner=np.true_divide(1-self.wide_range_corner,2); ### empty number of left and right colns
		self.offset=0.2
		self.img=img

		self.height_range=1 ### how much rows to save from bottom 

		#print 1



	def mask(self,img,station_F):  
	###  make mask on image, so that only shows middle potion of img
	###  if at station F, then only show left portion of img becasue Kinnect camera is at right side 
	###   input: img= RGB img;  station_F= if the task is at station_F

		# try:
			size=np.shape(img)
			row=size[0]
			coln=size[1]
			#print row,coln
			mask=np.zeros([row,coln],np.uint8)
			#print np.shape(mask)
			#print np.shape(mask[:,0])
			
			if station_F==2:
				for i in np.arange(int(self.left_coln_cornerF*coln),int((self.left_coln_cornerF+self.wide_range_cornerF)*coln)):  ## can be optimzied here  
					mask[:,i]=255
				for j in np.arange(0,int(row*(1-self.height_range_cornerF))):  ##3  can be optimized here 
					# print int(self.left_coln*coln)
					# print int((self.left_coln+self.wide_range)*coln)
					mask[j,int(self.left_coln_cornerF*coln):int((self.left_coln_cornerF+self.wide_range_cornerF)*coln)]=0
			elif station_F ==1:
				for i in np.arange(int(self.left_coln_cornerE*coln),int((self.left_coln_cornerE+self.wide_range_cornerE)*coln)):  ## can be optimzied here  
					mask[:,i]=255
				for j in np.arange(0,int(row*(1-self.height_range_cornerE))):  ##3  can be optimized here 
					# print int(self.left_coln*coln)
					# print int((self.left_coln+self.wide_range)*coln)
					mask[j,int(self.left_coln_cornerE*coln):int((self.left_coln_cornerE+self.wide_range_cornerE)*coln)]=0

			else:
				for i in np.arange(int(self.left_coln*coln),int((self.left_coln+self.wide_range)*coln)):  ## can be optimzied here  
					mask[:,i]=255
				for j in np.arange(0,int(row*(1-self.height_range))):  ##3  can be optimized here 
					# print int(self.left_coln*coln)
					# print int((self.left_coln+self.wide_range)*coln)
					mask[j,int(self.left_coln*coln):int((self.left_coln+self.wide_range)*coln)]=0
			
			masked_img=cv2.bitwise_and(img,img,mask=mask)
			###### helper function ##########
			# cv2.imshow('result',masked_img)
			# cv2.waitKey(0)
			##### ends ##############
			return masked_img
		# except:
		# 	print "[rgb_processing] mask funciton Error"


		

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
				# cv2.drawContours(img,[bound],0,(0,255,0),2)
			elif shape=="circle":
				bound = cv2.fitEllipse(cnt)
			###### helper lines: to see ....##########
				# cv2.ellipse(img,bound,(0,255,0),2)
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
			print label_length
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
			# final_mask=np.uint8(final_mask)
			# masked_img=cv2.bitwise_and(binary_img,binary_img,mask=final_mask)
			# cv2.imshow("result3",masked_img)
			# cv2.waitKey(20)
			#print goodLabels
			#print stats
			#print np.max(labels)
			#print final_mask
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
		if params.showImg==1:
			cv2.imshow("result1",img_hsv)
			cv2.waitKey(20)
		#########################
		mask=cv2.inRange(img_hsv,low_th,high_th) 
		#cv2.imshow("result2",mask)
		#cv2.waitKey(200)
		#print 3
		##### helper line ########
		if params.showImg==1:
			img=cv2.bitwise_and(img_hsv,img_hsv,mask=mask)
			cv2.imshow("result2",img)
			cv2.waitKey(20)
		#######################
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
		self.wide_range=0.23;   ### how much colns to save   0.3
		self.left_coln=np.true_divide(1-self.wide_range+0.21,3); ### +0.2 empty number of left and right colns ### was 2, changed it to 3 due to offset of kinect camera 
		self.height_range=0.5 ### how much rows to save from bottom 



		self.wide_range_cornerE=0.3;  ### how much colns to save 
		self.left_coln_cornerE=np.true_divide(1-self.wide_range_cornerE-0.2,2); ### empty number of left and right colns
		self.height_range_cornerE=0.5 ### how much rows to save from bottom 


		self.wide_range_cornerF=0.22;  ### how much colns to save 
		self.left_coln_cornerF=np.true_divide(1-self.wide_range_cornerF-0.08,2); ### empty number of left and right colns
		self.height_range_cornerF=0.5 ### how much rows to save from bottom 
		
		self.offset=0.2
		self.img=img

####### helper functions ############
	def find_contours_thresh(self,mask,area_th,img,kernel,iterations):
		#### function: this function find contour in a mask that has largest area  
		#### mask: 0,1 binary img 
		#### area_th: area threshhold for contour 
		#### img: RGB img for drawing 
		#### kernel: kernel for dilation 
		#### iterations: iteartions for dilation
		#### return: filtered contours  
				mask=np.array(mask,dtype=np.uint8)
				print "mask",np.sum(mask)
				#kernel=np.ones((9,9),np.uint8)
				mask=cv2.dilate(mask,kernel,iterations=iterations)
				print "mask_dialted",np.sum(mask)
				# cv2.imshow("dialted",mask)
				# cv2.waitKey(20)

				_,contours,_= cv2.findContours(mask, 1, 2)
				#print contours[0]
				#cnts= [cnt for cnt in contours if cv2.contourArea(cnt)>area_th]
				length=len(contours)
				print "contour_number",length
				if length==0 or length==1:  ### if no contour find 
					#print 1 
					return None
				else:  
					if length>2:
						area_max=[cv2.contourArea(contours[0]),cv2.contourArea(contours[1])]
						#print area_max
						cnts=[contours[0],contours[1]]
						i=0
						for cnt in contours[2:]: 
							area2=cv2.contourArea(cnt)
							if area2> area_max[0] or area2>area_max[1]:							
								INDEX=area_max.index(min(area_max))
								area_max[INDEX]=area2
								cnts[INDEX]=cnt

					else:
						cnts=[contours[0],contours[1]]
						#print cnt
						area_max=[cv2.contourArea(cnts[0]),cv2.contourArea(cnts[1])]
					###### helper function ######
					# cv2.drawContours(img,cnt, -1, (0,255,0), 3)
					# cv2.imshow("contours",img)
					# cv2.waitKey(20)
					# print area_max
					###### helper function ends ######
					#print area_max
					if area_max[0]>area_th and area_max[1]>area_th:
						return cnts 
					else:
						#print 1
						return None


	def find_three_contours_thresh(self,mask,area_th,img,kernel,iterations,breaker):
		#### function: this function find contour in a mask that has largest area  
		#### mask: 0,1 binary img 
		#### area_th: area threshhold for contour 
		#### img: RGB img for drawing 
		#### return: filtered contours  
			mask=np.uint8(mask)
			mask=cv2.dilate(mask,kernel,iterations=iterations)
			_,contours,_= cv2.findContours(mask, 1, 2)
			##### helper line #########
			# cv2.drawContours(img, contours, -1, (0,255,0), 3)
			# cv2.imshow("contours"+str(breaker),img)
			# cv2.waitKey(20)
			##### helper line ends ########
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
						area=cv2.contourArea(cnt)
						if area<10000:
							cnts.append(cnt)
							cnt_area[str(i)]=area
							print "contour", cnt_area[str(i)]
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
							print "[rgb_processing/find_three_contours_thresh]: I only find {} breakers".format(len(cnts_chosen))
						else: 
							print "[rgb_processing/find_three_contours_thresh]: I only find {} black_box".format(len(cnts_chosen)) 
						return None 
					else:
						if breaker==1:
							print "[rgb_processing/find_three_contours_thresh]: I find 3 brekers"
						else: 
							print "[rgb_processing/find_three_contours_thresh]: I find 3 black_box" 
						return cnts_chosen
					##### helper function ends ######
					# cv2.drawContours(img, cnts_chosen, -1, (0,255,0), 3)
					# cv2.imshow("contours"+str(breaker),img)
					# cv2.waitKey(20)
					###### helper function ends ######
				else:
					if breaker==1:
						print "[rgb_processing/find_three_contours_thresh]: I only find {} brekers".format(length)
					else:
						print "[rgb_processing/find_three_contours_thresh]: I only find {} black_box".format(length)
					return None




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

		masked_img=cv2.bitwise_and(img,img,mask=mask)
		# ###### helper function ##########
		# cv2.imshow('black_box',masked_img)
		# cv2.waitKey(20)
		##### ends ##############
		return masked_img		




	def black_box_proc(self,img,area_th,Type):
			######## the function process breaker 
			####### input: RGB image 
			#cv2.imshow("result4",img)
			#cv2.waitKey(0)
			if Type==41:
				low_th=params.breaker_A_blk_box_low_th      ## 0,0,0
				high_th=params.breaker_A_blk_box_high_th ##110,110,110
			else:
				low_th=params.breaker_B_blk_box_low_th      ## 0,0,0
				high_th=params.breaker_B_blk_box_high_th ##110,110,110
			##low_th=np.array([10,10,10])
			#high_th=np.array([15,15,15])
			mask=self.th_hsv(img,low_th,high_th)

			#cv2.imshow("masked_img_black_box2",mask)
			#cv2.waitKey(0)

			target_mask=self.findTarget(mask,area_th)
			
			if target_mask is None:
				print "[rgb_processing/black_box_proc]: didn't find black_boxes"
				return None 
				#cv2.imshow("mask",target_mask)
		  		#cv2.waitKey(200)
		  	##### helper line ###############
		  	#print 1
		  	#print 1
			# mask_final=np.uint8(target_mask)

			# # cv2.imshow("masked_img_black_box2q",img)
			# # cv2.waitKey(20)
			
			# masked_img=cv2.bitwise_and(img,img,mask=mask_final)
			# cv2.imshow("masked_img_black_box",masked_img)
			# cv2.waitKey(20)
			###############################
			return target_mask



	def breaker_status(self,breakers_info,black_boxes_info,Type):
			######## this function determines if a beaker is on or off 
			######## input: breakers_info: dictionanry of berakers_info from left to right 
			#######  black_boxes_info: dictionanry of black_boxes_info from left to right
			####### return: breaker state: up or down
			states=[] 
			for i in np.arange(3):
				breaker_Cy=breakers_info[i]["Cy"]
				box_Cy=black_boxes_info[i]["Cy"]
				print breaker_Cy, box_Cy
				if Type==41:
					print abs(breaker_Cy-box_Cy)
					if abs(breaker_Cy-box_Cy)<5: 
						state=-1 ### down 
					else:
						state=1 ### up
				else:
					print "difference", abs(breaker_Cy-box_Cy) 
					if abs(breaker_Cy-box_Cy)<10: 
						state=1 ### up 
					else:
						state=-1 ### down
				states.append(state)
			return states	




	def find_rect_centers(self,cnts,img):
			##### ths function find center of a bounding box ######
			### input: cnt: one contour 
			#### img: orignal rgb img for drawing 
			#### return: center of a bounding box  
			centers=[] 
			for cnt in cnts:	
				rect = cv2.minAreaRect(cnt)
				box = cv2.boxPoints(rect)
				Cx=np.average(box[:,0])  ### find X center of box 
				Cy=np.average(box[:,1])  ### fiind Y center of box 
				centers.append([Cx,Cy])
				#### helper function ####
				# int_box = np.int0(box)
				# cv2.drawContours(img,[int_box],0,(0,0,255),2)
				# cv2.imshow("rect",img)
				# cv2.waitKey(20)
				#### ends ####

			return centers




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
					breaker_info["Cy"]=Cy
					breaker_info["box"]=box
					breaker_info["cnt"]=cnt 
					breaker_list.append(breaker_info)
					 
				#### helper function ####
				# 	int_box = np.int0(box)
				# 	cv2.drawContours(img,[int_box],0,(0,0,255),2)
				# cv2.imshow("rect"+str(breaker),img)
				# cv2.waitKey(20)
				# #### ends ####
				breaker_list=sorted(breaker_list, key=lambda k: k["Cx"]) ### sort dictionary so that from left to right
				return breaker_list




	def find_contour_thresh(self,mask,area_th,img,kernel,iterations):
		#### function: this function find contour in a mask that has largest area  
		#### mask: 0,1 binary img 
		#### area_th: area threshhold for contour 
		#### img: RGB img for drawing 
		#### kernel: kernel for dilation 
		#### iterations: iteartions for dilation
		#### return: filtered contours  
				mask=np.array(mask,dtype=np.uint8)
				#print "mask",np.sum(mask)
				#kernel=np.ones((9,9),np.uint8)
				mask=cv2.dilate(mask,kernel,iterations=iterations)
				#print "mask_dialted",np.sum(mask)
				# cv2.imshow("dialted",mask)
				# cv2.waitKey(20)

				_,contours,_= cv2.findContours(mask, 1, 2)
				#print contours[0]
				#cnts= [cnt for cnt in contours if cv2.contourArea(cnt)>area_th]
				length=len(contours)
				#print "contour_number",length
				if length==0:  ### if no contour find 
					#print 1 
					return None
				else:  
					if length>1:
						area_max=cv2.contourArea(contours[0])
						#print area_max
						cnt=contours[0]
						i=0
						for cnts in contours: 
							area2=cv2.contourArea(cnts)
							if area2>area_max: 
								area_max=area2
								cnt=cnts
							i+=1
					else:
						cnt=contours[0]
						#print cnt
						area_max=cv2.contourArea(cnt)
					###### helper function ######
					# cv2.drawContours(img,cnt, -1, (0,255,0), 3)
					# cv2.imshow("contours",img)
					# cv2.waitKey(20)
					#print cnn
					###### helper function ends ######
					#print area_max
					if area_max>area_th:
						return cnt 
					else:
						#print 1
						return None 
	
########## proc each acutaor ###############

	def breaker_proc(self,img,area_th,Type):
		######## the function process breaker 
		####### input: RGB image 
		#cv2.imshow("result4",img)
		#cv2.waitKey(0)
		img_up=copy.deepcopy(img)
		img_special=copy.deepcopy(img)
		if Type==41: #### Type A breaker 
			low_th=params.breaker_A_low_th_up   ## up 
			high_th=params.breaker_A_high_th_up
			mask=self.th_hsv(img,low_th,high_th)

			low_th_up=params.breaker_A_low_th_down   ##down 
			high_th_up=params.breaker_A_high_th_down
			mask_up=self.th_hsv(img_up,low_th_up,high_th_up)


			low_th_special=params.breaker_A_low_special
			high_th_special=params.breaker_A_high_special
			mask_special=self.th_hsv(img_special,low_th_special,high_th_special)
			
			mask=np.uint8(np.logical_or(mask,mask_up))
			mask=np.uint8(np.logical_or(mask,mask_special))
		
		else: #### Type B breaker 
			low_th=params.breaker_B_low_th_up
			high_th=params.breaker_B_high_th_up
			mask=self.th_hsv(img,low_th,high_th)

			low_th_up=params.breaker_B_low_th_down
			high_th_up=params.breaker_B_high_th_down
			mask_up=self.th_hsv(img_up,low_th_up,high_th_up)

			low_th_special=params.breaker_B_low_special
			high_th_special=params.breaker_B_high_special
			mask_special=self.th_hsv(img_special,low_th_special,high_th_special)

			
			mask=np.uint8(np.logical_or(mask,mask_up))
			mask=np.uint8(np.logical_or(mask,mask_special))


		target_mask=self.findTarget(mask,area_th)
		##### helper line ###############
		# mask_final=np.uint8(target_mask)
		# masked_img=cv2.bitwise_and(img,img,mask=mask_final)
		# cv2.imshow("result4",masked_img)
		# cv2.waitKey(20)
		###############################
		########### helper linee ##########
		# 	cv2.imshow("mask",target_mask)
		# 	cv2.waitKey(20)
		# cv2.imshow("mask",target_mask)
		# cv2.waitKey(20)	
	  	####### helper line ends ########
	  	#print target_mask
	  	return target_mask

	def find_each_mask_breaker(self,breaker_infos,image):
		size=np.shape(image)
		row=size[0]
		coln=size[1]
		breaker_masks=[]
		for breaker_info in breaker_infos:
			mask=np.zeros([row,coln],np.uint8)
			cnt=breaker_info["cnt"]
			x,y,w,h=cv2.boundingRect(cnt)
			mask[x:x+w,y:y+h]=255
			breaker_masks.append(mask)
		return breaker_masks






			

	def breaker_status_proc(self,breaker_mask,img_visual,Type):
		cnt_area_th=10
		iterations=3
		kernel=np.ones((3,3),np.uint8)
		img_process=copy.deepcopy(img_visual)
		cnts=self.find_three_contours_thresh(breaker_mask,cnt_area_th,img_visual,kernel,iterations,breaker=1)
		if cnts is None: 
			return None,None
		else: 
			breaker_info=self.find_multi_rect_centers(cnts,img_visual,breaker=1)
			breaker_masks=self.find_each_mask_breaker(breaker_info,img_process)
			#print "length:",len(breaker_masks)
			#return None
			
			masked_image=self.get_blackBox_ROI(breaker_info,img_process)
			black_box_aera_th=50
			black_box_mask=self.black_box_proc(masked_image,black_box_aera_th,Type)
			if black_box_mask is not None:
				cnt_area_th=100
				iterations=1
				kernel=np.ones((3,3),np.uint8)
				cnts=self.find_three_contours_thresh(black_box_mask,cnt_area_th,img_visual,kernel,iterations,breaker=0)
				if cnts is None: 
					print 111111111111111111
					return None,None
				else :
					#print 11111111111111111111111111
					black_box_info=self.find_multi_rect_centers(cnts,img_visual,breaker=0)
					breaker_states=self.breaker_status(breaker_info,black_box_info,Type)
					print breaker_states
					return breaker_masks,breaker_states
			else:
				print "[rgb_processing/breaker_status_proc]: didn't find black_box_mask"
				return None,None


	def shuttle_proc(self,img,area_th,station_F):
			######## the function process breaker 
			####### input: RGB image 
			#cv2.imshow("result4",img)
			#cv2.waitKey(0)
			
			#low_th=np.array([70,200,200])
			#high_th=np.array([150,255,255])
			print "station", station_F
			if station_F==1:  ### for shuttle_proc at F 
				#low_th=np.array([80,150,100]) ## day: 100 150 50
				#high_th=np.array([130,220,150]) ## day: 150 220 100 
				low_th=params.shuttle_low_th_F
				high_th=params.shuttle_high_th_F
			else:
				#low_th=np.array([100,100,100]) ## 100 200 100
				#high_th=np.array([150,240,240]) ## 150 255 200
				low_th=params.shuttle_low_th
				high_th=params.shuttle_high_th


			mask=self.th_hsv(img,low_th,high_th)
			target_mask=self.findTarget(mask,area_th)
			##### helper line ###############
			# if target_mask is not None:
			# 	mask_final=np.uint8(target_mask)
			# 	masked_img=cv2.bitwise_and(img,img,mask=mask_final)
			# 	cv2.imshow("result4",masked_img)
			# 	cv2.waitKey(20)
			##############################
		  	# target_mask=np.uint8(target_mask)
		  	# cv2.imshow("mask",target_mask)
		  	# cv2.waitKey(20)	
		  	return target_mask  ## can be mask or None 

	def shuttle_sub_proc(self,img,target_mask_valve,blob_th_white,area_th_white,area_th_valve,station_F):
		img_visual=copy.deepcopy(img)
		
		##### white part ROI #### 
		iterations=3
		kernel=np.ones((5,5),np.uint8)
		cnt_valve=self.find_contour_thresh(target_mask_valve,area_th_valve,img_visual,kernel,iterations)
		if cnt_valve is not None:
			x,y,w,h = cv2.boundingRect(cnt_valve)
			#### helper function ####
			# cv2.rectangle(img_visual,(x,y),(x+w,y+h),(0,255,0),2)
			# cv2.imshow("unroated_rect",img_visual)
			# cv2.waitKey(20)
			#### ends ####
			### make mask###
			size=np.shape(img)
			row=size[0]
			coln=size[1]
			mask=np.zeros([row,coln],np.uint8)
			### ends ###

			scale_width=2
			scale_heigth=2
			x_start=int(x-w*scale_width)
			x_end=int(x+2*w*scale_width)
			y_start=int(y-h*scale_heigth)
			y_end=int(y+2*h*scale_heigth)
			mask[y_start:y_end+1,x_start:x_end+1]=255 
			masked_img=cv2.bitwise_and(img,img,mask=mask)
			####### helper function #########
			# cv2.imshow("masked_img",masked_img)
			# cv2.waitKey(20)
			#### ends ####
			##### find white #########
			if station_F==1: ## if at station E
				#low_th=np.array([50,0,200])
				#high_th=np.array([100,50,255])
				low_th=params.shuttle_white_low_th_F
				high_th=params.shuttle_white_high_th_F
			else:
				#low_th=np.array([50,0,220])
				#high_th=np.array([100,50,255])
				low_th=params.shuttle_white_low_th
				high_th=params.shuttle_white_high_th


			mask=self.th_hsv(masked_img,low_th,high_th)
			if mask is None: 
				print "[rgb_processing/shuttle_sub_proc] I can't find white part of shuttle valve in hsv"
				return None 
			else:
				target_mask=self.findTarget(mask,blob_th_white)
				if target_mask is None: 
					print "[rgb_processing/shuttle_sub_proc] I can't find white part of shuttle valve"
					return None
				else: 
					iterations=3
					kernel=np.ones((7,7),np.uint8)
					cnts_white=self.find_contours_thresh(target_mask,area_th_white,img_visual,kernel,iterations)
					
					if cnts_white is None or cnt_valve is None: 
						return None 
					else: 
						center_white=self.find_rect_centers(cnts_white,img_visual)
						print center_white[0][0], center_white[1][0]
						#distance=np.linalg.norm(np.array([center_white[0][0],center_white[0][1]])-np.array([center_white[1][0],center_white[1][1]]))
						distance=abs(center_white[0][0]-center_white[1][0])
						print distance  
						if distance>30: ## white part and valve are seperate ## 63 for horizontal valve  
							print "[rgb_processing/spigot_sub_proc] The shuttle valvue is horizontal"
							return -1
						else: 
							print "[rgb_processing/spigot_sub_proc] The shuttle valvue is vertial"
							return 1
						# print center_white[0][1], center_valve[0][1] 
						# if center_white[0][1]>center_valve[0][1]: ## white part and valve are seperate 
						# 	print "[rgb_processing/spigot_sub_proc] The shuttle valvue is horizontal"
						# 	return 2 
						# else: 
						# 	print "[rgb_processing/spigot_sub_proc] The spigot valvue is vertical"
					
		else: 
			print "[rgb_processing/spigot_sub_proc] I can't find valve contour"
			return None



	def spigot_proc(self,img,area_th,station_F):
		if station_F==1:
			low_th=params.spigot_low_th_F ## 100 200 100
			high_th=params.spigot_high_th_F ## 150 255 200
		else: 
			low_th=params.spigot_low_th ## 100 200 100
			high_th=params.spigot_high_th ## 150 255 200

		mask=self.th_hsv(img,low_th,high_th)
		target_mask=self.findTarget(mask,area_th)
		##### helper line ###############
		# if target_mask is not None:
		# 	mask_final=np.uint8(target_mask)
		# 	masked_img=cv2.bitwise_and(img,img,mask=mask_final)
		# 	cv2.imshow("result4",masked_img)
		# 	cv2.waitKey(20)
		#############################
		return target_mask


	def spigot_sub_proc(self,img,target_mask_valve,blob_th_white,area_th_white,area_th_valve,station_F):
		img_visual=copy.deepcopy(img)
		
		##### white part ROI #### 
		iterations=3
		kernel=np.ones((5,5),np.uint8)
		cnt_valve=self.find_contour_thresh(target_mask_valve,area_th_valve,img_visual,kernel,iterations)
		if cnt_valve is not None:
			x,y,w,h = cv2.boundingRect(cnt_valve)
			#### helper function ####
			# cv2.rectangle(img_visual,(x,y),(x+w,y+h),(0,255,0),2)
			# cv2.imshow("unroated_rect",img_visual)
			# cv2.waitKey(20)
			#### ends ####
			### make mask###
			size=np.shape(img)
			row=size[0]
			coln=size[1]
			mask=np.zeros([row,coln],np.uint8)
			### ends ###

			scale_width=0.8
			scale_heigth=1.8
			x_start=int(x-w*scale_width)
			x_end=int(x+2*w*scale_width)
			y_start=int(y-h*scale_heigth)
			y_end=int(y+2*h*scale_heigth)
			mask[y_start:y_end+1,x_start:x_end+1]=255 
			masked_img=cv2.bitwise_and(img,img,mask=mask)
			####### helper function #########
			# cv2.imshow("masked_img",masked_img)
			# cv2.waitKey(20)
			#### ends ####
			

			



			##### find white #########
			if station_F==1:
				low_th=params.spigot_white_low_th_F        ## day  [50,0,200] night [50,0,230]
				high_th=params.spigot_white_high_th_F     ## day  [120,50,255]
			else:
				low_th=params.spigot_white_low_th        ## day  [50,0,200] night [50,0,230]
				high_th=params.spigot_white_high_th     ## day  [120,50,255]

			mask=self.th_hsv(masked_img,low_th,high_th)
			if mask is None: 
				print "[rgb_processing/spigot_sub_proc] I can't find white part of spigot valve in hsv"
				return None 
			else:
				target_mask=self.findTarget(mask,blob_th_white)
				if target_mask is None: 
					print "[rgb_processing/spigot_sub_proc] I can't find white part of spigot valve"
					return None
				else: 
					iterations=3
					kernel=np.ones((3,3),np.uint8)
					cnt_white=self.find_contour_thresh(target_mask,area_th_white,img_visual,kernel,iterations)
					
					if cnt_white is None or cnt_valve is None: 
						return None 
					else: 
						center_white=self.find_rect_centers(cnt_white,img_visual)
						center_valve=self.find_rect_centers(cnt_valve,img_visual)
						#distance=np.linalg.norm(np.array([center_white[0][0],center_white[0][1]])-np.array([center_valve[0][0],center_valve[0][1]]))
						# print distance  ## 84.5 in normal for L-shape  ###  30 in normal distance for small-shape
						# if distance>50: ## white part and valve are seperate 
						# 	print "[rgb_processing/spigot_sub_proc] The spigot valvue is L shape"
						# 	return 2 
						# else: 
						# 	print "[rgb_processing/spigot_sub_proc] The spigot valvue is small shape"
						# 	return 1 
						print center_white[0][1], center_valve[0][1] 
						if center_white[0][1]>center_valve[0][1]: ## white part and valve are seperate 
							print "[rgb_processing/spigot_sub_proc] The spigot valvue is L shape"
							return -1 
						else: 
							print "[rgb_processing/spigot_sub_proc] The spigot valvue is small shape"
							return 1
					
		else: 
			print "[rgb_processing/spigot_sub_proc] I can't find valve contour"
			return None



	
	def orange(self,img,area_th,station_F):
		if station_F==1:
			low_th=params.orange_low_th_F    # 0,130,200
			high_th=params.orange_high_th_F  # 20,200,255
		else:
			low_th=params.orange_low_th   # 0,130,200
			high_th=params.orange_high_th  # 20,200,255

		mask=self.th_hsv(img,low_th,high_th)
		mask_final=np.uint8(mask)
		target_mask=self.findTarget(mask,area_th)
		##### helper line ###########
		# if target_mask is not None:
		# 	mask_final=np.uint8(target_mask)
		# 	masked_img=cv2.bitwise_and(img,img,mask=mask_final)
		# 	cv2.imshow("result4",masked_img)
		# 	cv2.waitKey(20)
		#############################
		return target_mask

	
		#################################################


	def locate_actuators(self,Type,station_F,img):
	#### process different type of actuato
		#### input: img: RGB img
		#### type: actutator type  1: B , 2: vertial 3: small 4: orange 5: L_shape 6: A  7:horizontal
		##### station_F: if at station F 
		#print 1 
		img1=copy.deepcopy(img)
		img2=copy.deepcopy(img) ## for sub proc 
		masked_img=self.mask(img1,station_F)
		#print np.shape(img)
		
		if Type==1: ### shuttle 
			blob_area_th=30
			target_mask=self.shuttle_proc(masked_img,blob_area_th,station_F)
			if target_mask is None: 
				print "[rgb_processing/locate_actuators]: can't find shuttle valve"
				subType=None
			else: 
				#subType=None
				area_th_white=10
				area_th_valve=20
				blob_th_white=50
				#subType=None
				masked_img=self.mask(img2,station_F)
				subType=self.shuttle_sub_proc(masked_img,target_mask,blob_th_white,area_th_white,area_th_valve,station_F)     
				if subType is None: 
					print "[rgb_processing/locate_actuators]: can't find shuttle valve subtype"
		


		elif Type==2: ### organe 
			blob_area_th=200
			target_mask=self.orange(masked_img,blob_area_th,station_F)
			if target_mask is None: 
				print "[rgb_processing/locate_actuators]: can't find orange valve"
			subType=None


		
		elif Type==3: ### spigot valve 
			print "spigot test" 
			blob_area_th=50
			target_mask=self.spigot_proc(masked_img,blob_area_th,station_F)
			if target_mask is None: 
				print "[rgb_processing/locate_actuators]: can't find spigot valve"
				subType=None
			else: 
				#subType=None
				area_th_white=10
				area_th_valve=20
				blob_th_white=20
				#subType=None
				masked_img=self.mask(img2,station_F)
				subType=self.spigot_sub_proc(masked_img,target_mask,blob_th_white,area_th_white,area_th_valve,station_F)     
				if subType is None: 
					print "[rgb_processing/locate_actuators]: can't find spigot valve subtype"
		

		elif Type==41 or Type==42:   ### breaker		
			#img3=copy.deepcopy(img) 
			blob_area_th=10
			target_mask=self.breaker_proc(masked_img,blob_area_th,Type)
			subType=None ## breaker doesn't have sub type
			breaker_states=None ## initialize breaker_states 
			if target_mask is None: 
				print "[rgb_processing/locate_actuators]: can't find breaker"
			else:
				target_mask,breaker_states=self.breaker_status_proc(target_mask,img2,Type)



		else:
			print "[rgb_processing/locate_actuators]: type must be within 1 to 4"
			target_mask=None
			subType=None 
			breaker_states=None
		if Type==41 or Type==42:
			return target_mask,subType,breaker_states
		else:
			return target_mask,subType,None




class pi_cam_process(rgb_process):
	def __init__(self,img):
		self.wide_range=0.8;  ### how much colns to save   0.3
		self.left_coln=np.true_divide(1-self.wide_range,2); ### empty number of left and right colns

		self.wide_range_corner=0.3;  ### how much colns to save 
		self.left_coln_corner=np.true_divide(1-self.wide_range_corner,2); ### empty number of left and right colns
		self.offset=0.2
		self.img=img

		self.height_range=1 ### how much rows to save from bottom 



	def green_tip_orange(self,img,area_th):
			######## the function process breaker 
			####### input: RGB image 
			#cv2.imshow("result4",img)
			#cv2.waitKey(0)
			# low_th=np.array([30,30,190])
			# high_th=np.array([100,100,230])

			low_th=params.pi_orange_green_tip_low_th
			high_th=params.pi_orange_green_tip_high_th

			mask=self.th_hsv(img,low_th,high_th)
			target_mask=self.findTarget(mask,area_th)
			if target_mask is None:
				size=np.shape(img)
				row=size[0]
				coln=size[1]
				target_mask=np.zeros([row,coln],np.uint8)
				#cv2.imshow("mask",target_mask)
		  		#cv2.waitKey(200)
		  	
			#### helper line ###############
			# if target_mask is not None:
			# 	mask_final=np.uint8(target_mask)
			# 	masked_img=cv2.bitwise_and(img,img,mask=mask_final)
			# 	cv2.imshow("result4",masked_img)
			# 	cv2.waitKey(20)
			###############################
			


		  	#cv2.imshow("mask",target_mask)
		  	#cv2.waitKey(200)	
		  	return target_mask



	def green_tip_small(self,img,area_th,subType):
			######## the function process spigot
			####### input: RGB image 
			#cv2.imshow("result4",img)
			#cv2.waitKey(0)
			if subType==0:
				low_th=params.pi_spigot_green_low_th_horizontal
				high_th=params.pi_spigot_green_high_th_horizontal
			elif subType==1:
				low_th=params.pi_spigot_green_low_th_vertical
				high_th=params.pi_spigot_green_high_th_vertical
			else: 
				print "please type either 0 (horizontal) or 1 (vertical) for subType"
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
			# cv2.waitKey(20)
			###############################
			


		  	#cv2.imshow("mask",target_mask)
		  	#cv2.waitKey(200)	
		  	return target_mask	  	



	def orange_proc(self,img,area_th):
			######## the function process breaker 
			####### input: RGB image 
			#cv2.imshow("result4",img)
			#cv2.waitKey(0)
			low_th=params.pi_orange_low_th
			high_th=params.pi_orange_high_th   ### b: 0 - 250 was also good 
			mask=self.th_hsv(img,low_th,high_th)
			target_mask=self.findTarget(mask,area_th)
			
			if target_mask is None:
				#print 1
				size=np.shape(img)
				row=size[0]
				coln=size[1]
				target_mask=np.zeros([row,coln],np.uint8)
				#cv2.imshow("mask",target_mask)
		  		#cv2.waitKey(200)
		  	##### helper line ###############
		  	#print 1
		  # 	if target_mask is not None:
				# mask_final=np.uint8(target_mask)
				# masked_img=cv2.bitwise_and(img,img,mask=mask_final)
				# cv2.imshow("masked_img",masked_img)
				# cv2.waitKey(20)
			###############################
		  	return target_mask


	def small_proc(self,img,area_th,subType):
			######## the function process small valve 
			####### input: RGB image 
			#cv2.imshow("result4",img)
			#cv2.waitKey(0)
			if subType==0:
				low_th=params.pi_spigot_low_th_horizontal
				high_th=params.pi_spigot_high_th_horizontal
			elif subType==1:
				low_th=params.pi_spigot_low_th_vertical
				high_th=params.pi_spigot_high_th_vertical
			else:
				print "please type either 0 (horizontal) or 1 (vertical) for subType"
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
			# cv2.waitKey(20)
			###############################
		  	return target_mask


	def switch_proc(self,img,area_th):
			######## the function process small valve 
			####### input: RGB image 
			#cv2.imshow("result4",img)
			#cv2.waitKey(0)
			print "im here"
			low_th=params.pi_shuttle_low_th
			high_th=params.pi_shuttle_high_th
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
			# cv2.waitKey(20)
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
			# cv2.waitKey(20)
			#### ends #########
			return (x,y)


	def find_ellipse_center(self,cnt,img):
		ellipse=cv2.fitEllipse(cnt)
		print ellipse
		# cv2.ellipse(img,ellipse,(0,0,255),2)
		# cv2.imshow("ellipse",img)
		# cv2.waitKey(20)
		return ellipse[0]


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
			if params.imshow_tip==1:
				int_box = np.int0(box)
				cv2.drawContours(img,[int_box],0,(0,0,255),2)
				cv2.imshow("rect",img)
				cv2.waitKey(20)
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
		# cv2.line(img,(int(x-vx*500),int(y-vy*500)),(int(x+vx*100),int(y+vy*100)),(0,0,255),2)
		# cv2.imshow("fitted_line",img)
		# cv2.waitKey(20)
		##### ends #########
		return (vx,vy,x,y)	


	# def find_line(self,cnt,center,img):
	# 	##### ths function find two poitns that fit a line of a contour ######
	# 	### input: cnt: one contour 
	# 	#### img: orignal rgb img for drawing 
	# 	#### return: direction vector (vx,vy) and a poit on the line (x,y)
	# 	vx,vy,_,_ = cv2.fitLine(cnt, cv2.DIST_L2,0,0.01,0.01)
	# 	x=center[0]
	# 	y=center[1]
	# 	#lefty = int((-x*vy/vx) + y)
	# 	#righty = int(((cols-x)*vy/vx)+y)
	# 	### helper function ####
	# 	print vx,vy,x,y
	# 	cv2.line(img,(int(x-vx*500),int(y-vy*500)),(int(x+vx*100),int(y+vy*100)),(0,0,255),2)
	# 	cv2.imshow("fitted_line",img)
	# 	cv2.waitKey(20)
	# 	##### ends #########
	# 	return (vx,vy,x,y)	


	def find_extrem_points(self,cnt,img):
		left=tuple(cnt[cnt[:,:,0].argmin()][0])
		right=tuple(cnt[cnt[:,:,0].argmax()][0])
		top=tuple(cnt[cnt[:,:,1].argmin()][0])
		bottom=tuple(cnt[cnt[:,:,1].argmax()][0])
		cv2.circle(img,left,5,(255,0,0))
		cv2.circle(img,right,5,(255,0,0))
		cv2.circle(img,top,5,(255,0,0))
		cv2.circle(img,bottom,5,(255,0,0))
		cv2.imshow("extremPoint",img)
		cv2.waitKey(20)
		return left 


	def unroated_box(self,cnt,img):
		###### this function find cebter of an unroated bounding box ########
		### input: cnt: one contour 
		#### img: orignal rgb img for drawing 
		#### return: center of a bounding box  
			x,y,w,h = cv2.boundingRect(cnt)
			#### helper function ####
			# if params.imshow_tip==1:
			# 	cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
			# 	cv2.imshow("unroated_rect",img)
			# 	cv2.waitKey(20)
			#### ends ####
			return (x,y,w,h)
		

	def find_contour_thresh(self,mask,area_th,img,kernel,iterations):
	#### function: this function find contour in a mask that has largest area  
	#### mask: 0,1 binary img 
	#### area_th: area threshhold for contour 
	#### img: RGB img for drawing 
	#### kernel: kernel for dilation 
	#### iterations: iteartions for dilation
	#### return: filtered contours  
			mask=np.array(mask,dtype=np.uint8)
			print "mask",np.sum(mask)
			#kernel=np.ones((9,9),np.uint8)
			mask=cv2.dilate(mask,kernel,iterations=iterations)
			print "mask_dialted",np.sum(mask)
			# cv2.imshow("dialted",mask)
			# cv2.waitKey(20)

			_,contours,_= cv2.findContours(mask, 1, 2)
			#print contours[0]
			#cnts= [cnt for cnt in contours if cv2.contourArea(cnt)>area_th]
			length=len(contours)
			print "contour_number",length
			if length==0:  ### if no contour find 
				#print 1 
				return None
			else:  
				if length>1:
					area_max=cv2.contourArea(contours[0])
					#print area_max
					cnt=contours[0]
					i=0
					for cnts in contours: 
						area2=cv2.contourArea(contours[i])
						if area2>area_max: 
							area_max=area2
							cnt=contours[i]
						i+=1
				else:
					cnt=contours[0]
					#print cnt
					area_max=cv2.contourArea(cnt)
				###### helper function ######
				# cv2.drawContours(img,cnt, -1, (0,255,0), 3)
				# cv2.imshow("contours",img)
				# cv2.waitKey(20)
				#print cnn
				###### helper function ends ######
				#print area_max
				if area_max>area_th:
					return cnt 
				else:
					#print 1
					return None 
	
		  	

	def calc_angle_circle(self,center,pt,img):
	#### this function calcualte anlge of line respect to +x axis in cw 
	#### input: center, pt2: two pts in tuple; center is the center of a circle 
	#### return anlge with respect to +x in cw  
			vec=np.array(pt)-np.array(center)
			angle=np.angle(vec[0]+vec[1]*1j,deg=True)
		##### helper function #####
			if params.imshow_tip==1:
				center_int=(int(center[0]),int(center[1]))
				pt_int=(int(pt[0]),int(pt[1]))
				cv2.line(img, center_int, pt_int, (255,0,0), 2)
				print angle
				cv2.imshow("Line",img)
				cv2.waitKey(20) 
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
			low_th=np.array([0,80,120])
			high_th=np.array([50,255,255])
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
			# cv2.waitKey(20)
			###############################
			return target_mask


	def find_three_contours_thresh(self,mask,area_th,img,kernel,iterations,breaker):
		#### function: this function find contour in a mask that has largest area  
		#### mask: 0,1 binary img 
		#### area_th: area threshhold for contour 
		#### img: RGB img for drawing 
		#### return: filtered contours  
			mask=np.uint8(mask)
			mask=cv2.dilate(mask,kernel,iterations=iterations)
			_,contours,_= cv2.findContours(mask, 1, 2)
			##### helper line #########
			# cv2.drawContours(img, contours, -1, (0,255,0), 3)
			# cv2.imshow("contours"+str(breaker),img)
			# cv2.waitKey(20)
			##### helper line ends ########
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
						print "contour", cnt_area[str(i)]
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
					# cv2.waitKey(20)
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
			# cv2.waitKey(20)
			# #### ends ####
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
		# cv2.waitKey(20)
		##### ends ##############
		return masked_img		


	def black_box_proc(self,img,area_th):
		######## the function process breaker 
		####### input: RGB image 
		#cv2.imshow("result4",img)
		#cv2.waitKey(0)
		low_th=np.array([100,0,0])
		high_th=np.array([150,100,150])
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

		# # cv2.imshow("masked_img_black_box2q",img)
		# # cv2.waitKey(20)
		
		# masked_img=cv2.bitwise_and(img,img,mask=mask_final)
		# cv2.imshow("masked_img_black_box",masked_img)
		# cv2.waitKey(20)
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


	def locate_green(self,type,img,subType,station_F=0):
			masked_img=self.mask(img,station_F)
			img_visual=copy.deepcopy(img)
			if type==2: ## orange valve

				####### find center of orange valve ########
				orange_area_th=10000
				orange_mask=self.orange_proc(masked_img,orange_area_th)  ### may want to dialte the mask

	
				cnt_area_th=10000
				iterations=3
				kernel=np.ones((9,9),np.uint8)
				cnt=self.find_contour_thresh(orange_mask,cnt_area_th,img_visual,kernel,iterations)
				if cnt is None: 
					print "[pi_cam_info]: I can't find organe valve" 
					return None 
				else:
					circle_center=self.find_circle_center(cnt,img_visual)
					x,y,w,h=self.unroated_box(cnt,img_visual)
					masked_img=self.get_ROI((x,y,w,h),img_visual)

				######## find center of green tip #########
					tip_area_th=400
					tip_mask=self.green_tip_orange(masked_img,tip_area_th)
					cnt_area_th=400
					iterations=1
					kernel=np.ones((1,1),np.uint8)
					cnts= self.find_contour_thresh(tip_mask,cnt_area_th,img_visual,kernel,iterations)
					if cnts is None: 
						print "[pi_cam_info]: I can't find green tip on small valve" 
						return None 
					else:					
						tips_center=self.find_rect_center(cnts,img_visual)	
						angle=self.calc_angle_circle(circle_center,tips_center,img_visual)
						print "[pi_cam_info] Orange valve orientation is",angle,"deg"
						return angle 

			elif type==3: ## spigot valve 
				small_area_th=10000
				small_mask=self.small_proc(masked_img,small_area_th,subType)

				iterations=3
				kernel=np.ones((13,13),np.uint8)
				cnt_area_th=10000
				cnt= self.find_contour_thresh(small_mask,cnt_area_th,img_visual,kernel,iterations)
				if cnt is None: 
					print "[pi_cam_info]: I can't find small blue valve" 
					return None 
				else: 
					circle_center=self.find_circle_center(cnt,img_visual)
					x,y,w,h=self.unroated_box(cnt,img_visual)
					masked_img=self.get_ROI((x,y,w,h),img_visual)

				#print tips_center  
					tip_area_th=400
					tip_mask=self.green_tip_small(masked_img,tip_area_th,subType)
					cnt_area_th=400
					iterations=2
					kernel=np.ones((5,5),np.uint8)
					cnt= self.find_contour_thresh(tip_mask,cnt_area_th,img_visual,kernel,iterations)
					if cnt is None: 
						print "[pi_cam_info]: I can't find green tip on small valve" 
						return None 
					else:
						tips_center=self.find_rect_center(cnt,img_visual)
						#tips_center=self.find_ellipse_center(cnt,img_visual)
						#tips_center=self.find_line(cnt,img_visual)
						#tips_center=self.find_extrem_points(cnt,img_visual)
						angle=self.calc_angle_circle(circle_center,tips_center,img_visual)
						print "[pi_cam_info] Small blue valve orientation is",angle,"deg"
						return angle

			elif type==1: ## shuttle valve
				masked_img=img  ## don't use cropped image 
				small_area_th=10000
				small_mask=self.switch_proc(masked_img,small_area_th)
				
				cnt_area_th=8000
				iterations=2
				kernel=np.ones((3,3),np.uint8)
				cnt= self.find_contour_thresh(small_mask,cnt_area_th,img_visual,kernel,iterations)
				
				if cnt is None: 
					print "[pi_cam_info]: I can't find horizontal blue switch" 
					return None 
				else: 
					vx,vy,_,_=self.find_line(cnt,img_visual)
					angle=np.angle(vx+vy*1j,deg=True)
					print "The swithc is ", angle, "deg" 
					return abs(angle) 
					#x,y,w,h=self.unroated_box(cnt,img_visual)
					#masked_img=self.get_ROI((x,y,w,h),img_visual)

			elif type==4: ## breaker 
				breaker_area_th=500
				breaker_mask=self.breaker_proc(masked_img,breaker_area_th)

				cnt_area_th=500
				iterations=3
				kernel=np.ones((3,3),np.uint8)
				cnts=self.find_three_contours_thresh(breaker_mask,cnt_area_th,img_visual,kernel,iterations,breaker=1)
				if cnts is None: 
					return None
				else: 
					breaker_info=self.find_multi_rect_centers(cnts,img_visual,breaker=1)
					#return None
					#### comment out, may use kinect to finish it #######
					masked_image=self.get_blackBox_ROI(breaker_info,img)
					black_box_aera_th=500
					black_box_mask=self.black_box_proc(masked_image,black_box_aera_th)

					cnt_area_th=500
					iterations=1
					kernel=np.ones((1,1),np.uint8)
					cnts=self.find_three_contours_thresh(black_box_mask,cnt_area_th,img_visual,kernel,iterations,breaker=0)
					if cnts is None: 
						return None
					else :
						black_box_info=self.find_multi_rect_centers(cnts,img_visual,breaker=0)
						breaker_states=self.breaker_status(breaker_info,black_box_info)
						print breaker_states
						#print black_box_info 
						return breaker_states



			







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