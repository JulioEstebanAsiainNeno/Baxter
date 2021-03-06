########################################################################
# JeffProd Simple Python Chess Program
########################################################################
# AUTHOR	: Jean-Francois GAZET
# WEB 		: http://www.jeffprod.com
# TWITTER	: @JeffProd
# MAIL		: jeffgazet@gmail.com
# LICENCE	: GNU GENERAL PUBLIC LICENSE Version 2, June 1991
########################################################################

############################################################################################
#########################################TRADUCCION#########################################
## Deplacements = Move									  ##
## Echiquier = ChessBoard								  ##
## Ep = En passant									  ##
## Cases = Square									  ##
## Prise = (Catch/Eat) a piece								  ##
#########################################TRADUCCION#########################################
############################################################################################

from piece import *
from datetime import datetime 
import time
import rospy
import baxter_interface
import math
import tf
import cv2
import cv
#import cv2.cv as cv
import cv_bridge
from sensor_msgs.msg import Image
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

class Engine:
    
    "The chess engine"
    
    ####################################################################
    
    def __init__(self, tab):
        
        self.MAX_PLY=32
        self.pv_length=[0 for x in range(self.MAX_PLY)]
        self.INFINITY=32000        
        self.init()
	
	self.tab = tab
	self.gripper = baxter_interface.Gripper("right")
	self.gripper.calibrate()
	self.gripper.open()
	self.limb = baxter_interface.Limb('right')
	self.limb2 = baxter_interface.Limb('left')
	self.piecePrise = Piece()
	self.castle_pos1 = Piece()
	self.castle_pos2 = Piece()
	self.en_passant = False
	self.boolc = False
	
    ####################################################################

    def mensaje_matriz_a_pose(self,T, frame):
        t = PoseStamped()
        t.header.frame_id = frame
        t.header.stamp = rospy.Time.now()
        translacion = tf.transformations.translation_from_matrix(T)
        orientacion = tf.transformations.quaternion_from_matrix(T)
        t.pose.position.x = translacion[0]
        t.pose.position.y = translacion[1]
        t.pose.position.z = translacion[2]
        t.pose.orientation.x = orientacion[0]
        t.pose.orientation.y = orientacion[1]
        t.pose.orientation.z = orientacion[2]
        t.pose.orientation.w = orientacion[3]        
        return t

    ####################################################################

    def mover_baxter(self, source_frame, trans, rot):

        nombre_servicio = '/ExternalTools/'+ 'right' +'/PositionKinematicsNode/IKService'
        servicio_ik = rospy.ServiceProxy(nombre_servicio,SolvePositionIK)
        frame = source_frame   

        # Promedio de velocidad del brazo
        self.limb.set_joint_position_speed(0.5)

        matrix = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans),
            tf.transformations.euler_matrix(rot[0],rot[1],rot[2])
            )
        
        rospy.wait_for_service(nombre_servicio,10)
        ik_mensaje = SolvePositionIKRequest()
        ik_mensaje.pose_stamp.append(self.mensaje_matriz_a_pose(matrix, frame))

        try:
            respuesta = servicio_ik(ik_mensaje)
        except:
            print "Movimiento no ejecutado"

        print respuesta.isValid[0]

        if respuesta.isValid[0] == True:
            movimiento =  dict(zip(respuesta.joints[0].name, respuesta.joints[0].position))
            self.limb.move_to_joint_positions(movimiento)
        else:
            print "Movimiento no ejecutado"

        #print respuesta.joints[0].position
        #print respuesta.joints[0].name

    ####################################################################
    
    def usermove(self,b,c):
        print "usermove"
        """Move a piece for the side to move, asked in command line.
        The command 'c' in argument is like 'e2e4' or 'b7b8q'.
        Argument 'b' is the chessboard.
        """
        
        if(self.endgame):
            self.print_result(b)
            return        
              
        # Testing the command 'c'. Exit if incorrect.
        chk=self.chkCmd(c)
        if(chk!=''):
            print(chk)
            return
            
        # Convert cases names to int, ex : e3 -> 44
        pos1=b.caseStr2Int(c[0]+c[1])
        pos2=b.caseStr2Int(c[2]+c[3])
        
	desvx1 = 0
	desvy1 = 0
	desvx2 = 0
	desvy2 = 0
	#Positivo va hacia la pizarra
	filo, fild = 0, 0
	fil1 = 0.0218
	fil2 = 0.020
	fil3 = 0.021
	fil4 = 0.021
	fil5 = 0.025
	fil6 = 0.025
	fil7 = 0.027
	fil8 = 0.028
	#Positivo va hacia la puerta
	colo, cold = 0, 0
	col1 = -0.014
	col2 = -0.0131
	col3 = -0.0116
	col4 = -0.011
	col5 = -0.00962
	col6 = -0.010
	col7 = -0.0126
	col8 = -0.0087
	#Positivo va hacia el techo
	alto, altd = 0, 0
	alt1 = 0.0001
	alt2 = 0
	alt3 = 0.001
	alt4 = 0.0005
	alt5 = 0.0015
	alt6 = 0.0005
	alt7 = -0.0022
	alt8 = -0.0022

	#offset fila y alto origen
	if pos1 > 0 and pos1 < 8:
		filo = fil1
		alto = alt1
	elif pos1 > 7 and pos1 < 16:
		filo = fil2
		alto = alt2
	elif pos1 > 15 and pos1 < 24:
		filo = fil3
		alto = alt3
	elif pos1 > 23 and pos1 < 32:
		filo = fil4
		alto = alt4
	elif pos1 > 31 and pos1 < 40:
		filo = fil5
		alto = alt5
	elif pos1 > 39 and pos1 < 48:
		filo = fil6
		alto = alt6
	elif pos1 > 47 and pos1 < 56:
		filo = fil7
		alto = alt7
	elif pos1 > 55 and pos1 < 64:
		filo = fil8
		alto = alt8
	#offset fila y alto destino
	if pos2 > 0 and pos2 < 8:
		fild = fil1
		altd = alt1
	elif pos2 > 7 and pos2 < 16:
		fild = fil2
		altd = alt2
	elif pos2 > 15 and pos2 < 24:
		fild = fil3
		altd = alt3
	elif pos2 > 23 and pos2 < 32:
		fild = fil4
		altd = alt4
	elif pos2 > 31 and pos2 < 40:
		fild = fil5
		altd = alt5
	elif pos2 > 39 and pos2 < 48:
		fild = fil6
		altd = alt6
	elif pos2 > 47 and pos2 < 56:
		fild = fil7
		altd = alt7
	elif pos2 > 55 and pos2 < 64:
		fild = fil8
		altd = alt8
	#offset columna origen
	if (pos1+1)%8 == 1:
		colo = col1
	elif (pos1+1)%8 == 2:
		colo = col2
	elif (pos1+1)%8 == 3:
		colo = col3
	elif (pos1+1)%8 == 4:
		colo = col4
	elif (pos1+1)%8 == 5:
		colo = col5
	elif (pos1+1)%8 == 6:
		colo = col6
	elif (pos1+1)%8 == 7:
		colo = col7
	elif (pos1+1)%8 == 0:
		colo = col8
	#offset columna destino
	if (pos2+1)%8 == 1:
		cold = col1
	elif (pos2+1)%8 == 2:
		cold = col2
	elif (pos2+1)%8 == 3:
		cold = col3
	elif (pos2+1)%8 == 4:
		cold = col4
	elif (pos2+1)%8 == 5:
		cold = col5
	elif (pos2+1)%8 == 6:
		cold = col6
	elif (pos2+1)%8 == 7:
		cold = col7
	elif (pos2+1)%8 == 0:
		cold = col8

	desvy1, desvy2 = colo, cold
	desvx1, desvx2 = filo, fild
	desvz1, desvz2 = alto, altd

        # Promotion asked ?
        promote=''
        if(len(c)>4):
            promote=c[4]
            if(promote=='q'):
                promote='q'
            elif(promote=='r'):
                promote='r'
            elif(promote=='n'):
                promote='n'
            elif(promote=='b'):
                promote='b'
            
        # Generate moves list to check 
        # if the given move (pos1,pos2,promote) is correct
        mList=b.gen_moves_list()
        
        # The move is not in list ? or let the king in check ?
        if(((pos1,pos2,promote) not in mList) or \
        (b.domove(pos1,pos2,promote)==False)):
            print("\n"+c+' : incorrect move or let king in check'+"\n")
            return
        # Display the chess board
        b.render()
        
        # Check if game is over
        self.print_result(b)
        
    ####################################################################

    def chkCmd(self,c):
        
        """Check if the command 'c' typed by user is like a move,
        i.e. 'e2e4','b7b8n'...
        Returns '' if correct.
        Returns a string error if not.
        """
        
        err=(
        'The move must be 4 or 5 letters : e2e4, b1c3, e7e8q...',
        'Incorrect move.'
        )        
        letters=('a','b','c','d','e','f','g','h')
        numbers=('1','2','3','4','5','6','7','8')
                
        if(len(c)<4 or len(c)>5):
            return err[0]
        
        if(c[0] not in letters):
            return err[1]
            
        if(c[1] not in numbers):
            return err[1]
            
        if(c[2] not in letters):
            return err[1]
            
        if(c[3] not in numbers):
            return err[1]
            
        return ''

    ####################################################################
    
    def search(self,b):
        
        """Search the best move for the side to move,
        according to the given chessboard 'b'
        """
        if(self.endgame):
            self.print_result(b)
            return
            
        # TODO
        # search in opening book

        self.clear_pv()
        self.nodes=0
        b.ply=0
        
        print("ply\tnodes\tscore\tpv")
        
        for i in range(1,self.init_depth+1):
           
            score=self.alphabeta(i,-self.INFINITY,self.INFINITY,b)

            print "{}\t{}\t{}\t".format(i,self.nodes,score/10), #print("{}\t{}\t{}\t".format(i,self.nodes,score/10),end='')
            
            # print PV informations : ply, nodes...
            j=0
            while(self.pv[j][j]!=0):
                c=self.pv[j][j]
                pos1=b.caseInt2Str(c[0])
                pos2=b.caseInt2Str(c[1])
                print "{}{}{}".format(pos1,pos2,c[2]), #print("{}{}{}".format(pos1,pos2,c[2]),end=' ')
                j+=1
            print

            # Break if MAT is found
            if(score>self.INFINITY-100 or score<-self.INFINITY+100):
                break

        # root best move found, do it, and print result
        best=self.pv[0][0]

	desvx1 = 0
	desvy1 = 0
	desvx2 = 0
	desvy2 = 0
	#Positivo va hacia la pizarra
	filo, fild = 0, 0
	fil1 = 0.0217
	fil2 = 0.0202
	fil3 = 0.021
	fil4 = 0.021
	fil5 = 0.025
	fil6 = 0.025
	fil7 = 0.027
	fil8 = 0.028
	#Positivo va hacia la puerta
	colo, cold = 0, 0
	col1 = -0.014
	col2 = -0.0131
	col3 = -0.0116
	col4 = -0.011
	col5 = -0.00962
	col6 = -0.010
	col7 = -0.0126
	col8 = -0.0087
	#Positivo va hacia el techo
	alto, altd = 0, 0
	alt1 = 0.0001
	alt2 = 0
	alt3 = 0.001
	alt4 = 0.0005
	alt5 = 0.0015
	alt6 = 0.0005
	alt7 = -0.0022
	alt8 = -0.0022

	#offset fila y alto origen
	if best[0] > 0 and best[0] < 8:
		filo = fil1
		alto = alt1
	elif best[0] > 7 and best[0] < 16:
		filo = fil2
		alto = alt2
	elif best[0] > 15 and best[0] < 24:
		filo = fil3
		alto = alt3
	elif best[0] > 23 and best[0] < 32:
		filo = fil4
		alto = alt4
	elif best[0] > 31 and best[0] < 40:
		filo = fil5
		alto = alt5
	elif best[0] > 39 and best[0] < 48:
		filo = fil6
		alto = alt6
	elif best[0] > 47 and best[0] < 56:
		filo = fil7
		alto = alt7
	elif best[0] > 55 and best[0] < 64:
		filo = fil8
		alto = alt8
	#offset fila y alto destino
	if best[1] > 0 and best[1] < 8:
		fild = fil1
		altd = alt1
	elif best[1] > 7 and best[1] < 16:
		fild = fil2
		altd = alt2
	elif best[1] > 15 and best[1] < 24:
		fild = fil3
		altd = alt3
	elif best[1] > 23 and best[1] < 32:
		fild = fil4
		altd = alt4
	elif best[1] > 31 and best[1] < 40:
		fild = fil5
		altd = alt5
	elif best[1] > 39 and best[1] < 48:
		fild = fil6
		altd = alt6
	elif best[1] > 47 and best[1] < 56:
		fild = fil7
		altd = alt7
	elif best[1] > 55 and best[1] < 64:
		fild = fil8
		altd = alt8
	#offset columna origen
	if (best[0]+1)%8 == 1:
		colo = col1
	elif (best[0]+1)%8 == 2:
		colo = col2
	elif (best[0]+1)%8 == 3:
		colo = col3
	elif (best[0]+1)%8 == 4:
		colo = col4
	elif (best[0]+1)%8 == 5:
		colo = col5
	elif (best[0]+1)%8 == 6:
		colo = col6
	elif (best[0]+1)%8 == 7:
		colo = col7
	elif (best[0]+1)%8 == 0:
		colo = col8
	#offset columna destino
	if (best[1]+1)%8 == 1:
		cold = col1
	elif (best[1]+1)%8 == 2:
		cold = col2
	elif (best[1]+1)%8 == 3:
		cold = col3
	elif (best[1]+1)%8 == 4:
		cold = col4
	elif (best[1]+1)%8 == 5:
		cold = col5
	elif (best[1]+1)%8 == 6:
		cold = col6
	elif (best[1]+1)%8 == 7:
		cold = col7
	elif (best[1]+1)%8 == 0:
		cold = col8

	desvy1, desvy2 = colo, cold
	desvx1, desvx2 = filo, fild
	desvz1, desvz2 = alto, altd

	self.piecePrise = b.get_piecePrise()
	posep = 0

	b.set_en_passant(False)
	print "en_passant = ", self.en_passant 
        b.domove(best[0],best[1],best[2])
	
	self.en_passant = b.get_en_passant()	
	print "en_passant = ", self.en_passant 
	
	if self.en_passant == True and b.side2move == 'noir':	
		posep = 8
	elif self.en_passant == True and b.side2move == 'blanc':
		posep = -8
	print "posep: ", posep

	###############################        
	## AGREGAR MOVIMIENTO BAXTER ##
	###############################
	print "1: Abrir Grip"	
	self.gripper.open()	
	self.mover_baxter('base',[0.537, -0.235 , -0.10],[-math.pi,0,math.pi])						#Posicion inicial
	#print self.limb.endpoint_pose()
	print "2: Posicion inicial"
	self.piecePrise = b.get_piecePrise()										#Pieza Comida
	print self.piecePrise.nom
	if(self.piecePrise.nom != '.'):											#Pieza Comida?
		self.mover_baxter('base',[self.tab[best[1]+posep][0]+desvx2, self.tab[best[1]+posep][1]+desvy2, -0.10],[-math.pi,0,math.pi])	#Arriba
		rospy.sleep(0.2)  #1
		self.mover_baxter('base',[self.tab[best[1]+posep][0]+desvx1, self.tab[best[1]+posep][1]+desvy1, -0.16],[-math.pi,0,math.pi])	#Mas abajo
		self.mover_baxter('base',[self.tab[best[1]+posep][0]+desvx2, self.tab[best[1]+posep][1]+desvy2, -0.205+desvz2],[-math.pi,0,math.pi])	#Baja
		self.gripper.close()												#La toma
		rospy.sleep(0.5)
		self.mover_baxter('base',[self.tab[best[1]+posep][0]+desvx1, self.tab[best[1]+posep][1]+desvy1, -0.16],[-math.pi,0,math.pi])	#Mas abajo
		self.mover_baxter('base',[self.tab[best[1]+posep][0]+desvx2, self.tab[best[1]+posep][1]+desvy2, -0.10],[-math.pi,0,math.pi])	#La levanta
		self.mover_baxter('base',[0.537, -0.700 , -0.10],[-math.pi,0,math.pi])						#La saca
		self.gripper.open()												#Abre
		self.mover_baxter('base',[0.537, -0.235 , -0.10],[-math.pi,0,math.pi])						#Inicial
	self.mover_baxter('base',[self.tab[best[0]][0]+desvx1, self.tab[best[0]][1]+desvy1, -0.08],[-math.pi,0,math.pi])	#Arriba de la pieza
	print "3: Arriba de la pieza"
	rospy.sleep(0.2)  #1
	self.mover_baxter('base',[self.tab[best[0]][0]+desvx1, self.tab[best[0]][1]+desvy1, -0.16],[-math.pi,0,math.pi])	#Mas abajo
	print "3: Mas abajo"
	rospy.sleep(0.5)
	self.mover_baxter('base',[self.tab[best[0]][0]+desvx1, self.tab[best[0]][1]+desvy1, -0.205+desvz1],[-math.pi,0,math.pi])	#En la pieza
	print "4: En la pieza"
	self.gripper.close()												#Agarra la pieza
	rospy.sleep(0.5)													#Espera
	print "5: Agarra la pieza"
	self.mover_baxter('base',[self.tab[best[0]][0]+desvx1, self.tab[best[0]][1]+desvy1, -0.16],[-math.pi,0,math.pi])	#Levanta la pieza
	print "6: Levanta la pieza"
	self.mover_baxter('base',[self.tab[best[0]][0]+desvx1, self.tab[best[0]][1]+desvy1, -0.10],[-math.pi,0,math.pi])	#Levanta la pieza
	print "6: Levanta la pieza"
	#fin posicion 1
	self.mover_baxter('base',[self.tab[best[1]][0]+desvx2, self.tab[best[1]][1]+desvy2, -0.10],[-math.pi,0,math.pi])	#Arriba del destino
	print "7: Arriba del destino"
	self.mover_baxter('base',[self.tab[best[1]][0]+desvx2, self.tab[best[1]][1]+desvy2, -0.16],[-math.pi,0,math.pi])	#Mas abajo
	print "7: Mas abajo"
	self.mover_baxter('base',[self.tab[best[1]][0]+desvx2, self.tab[best[1]][1]+desvy2, -0.205+desvz2],[-math.pi,0,math.pi])	#Baja al destino
	print "8: Baja al destino"
	self.gripper.open()
	rospy.sleep(0.5)												#Suelta la pieza
	print "9: Suelta la pieza"
	self.mover_baxter('base',[self.tab[best[1]][0]+desvx2, self.tab[best[1]][1]+desvy2, -0.16],[-math.pi,0,math.pi])	#Sube de nuevo
	rospy.sleep(0.2)
	print "10: Sube de nuevo"
	self.mover_baxter('base',[self.tab[best[1]][0]+desvx2, self.tab[best[1]][1]+desvy2, -0.10],[-math.pi,0,math.pi])	#Sube de nuevo
	rospy.sleep(0.2)
	print "10: Sube de nuevo"
	self.mover_baxter('base',[self.tab[best[1]][0]+desvx2, self.tab[best[1]][1]+desvy2, -0.00],[-math.pi,0,math.pi])	#Sube mas
	print "11: Sube mas"
	print "Enroque: ", self.castle_pos2.nom
	if self.castle_pos2.nom != '.':
		print "Entra al Enroque"
		print "castle_pos1 ", castle_pos1
		print "castle_pos2 ", castle_pos2
		self.mover_baxter('base',[self.tab[castle_pos1][0]+desvx1, self.tab[castle_pos1][1]+desvy1, -0.10],[-math.pi,0,math.pi])
		print "3: Arriba de la pieza"
		rospy.sleep(0.2)  #1
		self.mover_baxter('base',[self.tab[castle_pos1][0]+desvx1, self.tab[castle_pos1][1]+desvy1, -0.16],[-math.pi,0,math.pi])
		print "3: Mas abajo"
		rospy.sleep(0.5)
		self.mover_baxter('base',[self.tab[castle_pos1][0]+desvx1, self.tab[castle_pos1][1]+desvy1, -0.205+desvz1],[-math.pi,0,math.pi])
		print "4: En la pieza"
		self.gripper.close()
		rospy.sleep(0.5)
		print "5: Agarra la pieza"
		self.mover_baxter('base',[self.tab[castle_pos1][0]+desvx1, self.tab[castle_pos1][1]+desvy1, -0.16],[-math.pi,0,math.pi])
		print "3: Mas abajo"
		self.mover_baxter('base',[self.tab[castle_pos1][0]+desvx1, self.tab[castle_pos1][1]+desvy1, -0.10],[-math.pi,0,math.pi])
		print "6: Levanta la pieza"
		self.mover_baxter('base',[self.tab[castle_pos2][0]+desvx2, self.tab[castle_pos2][1]+desvy2, -0.10],[-math.pi,0,math.pi])
		print "7: Arriba del destino"
		self.mover_baxter('base',[self.tab[castle_pos2][0]+desvx2, self.tab[castle_pos2][1]+desvy2, -0.16],[-math.pi,0,math.pi])
		print "7: Mas abajo"
		self.mover_baxter('base',[self.tab[castle_pos2][0]+desvx2, self.tab[castle_pos2][1]+desvy2, -0.205+desvz2],[-math.pi,0,math.pi])
		print "8: Baja al destino"
		self.gripper.open()	
		rospy.sleep(0.5)
		print "9: Suelta la pieza"
		self.mover_baxter('base',[self.tab[castle_pos2][0]+desvx2, self.tab[castle_pos2][1]+desvy2, -0.16],[-math.pi,0,math.pi])
		print "10: Sube de nuevo"
		rospy.sleep(0.2)
		self.mover_baxter('base',[self.tab[castle_pos2][0]+desvx2, self.tab[castle_pos2][1]+desvy2, -0.10],[-math.pi,0,math.pi])
		print "10: Sube de nuevo"
		rospy.sleep(0.2)
		self.mover_baxter('base',[self.tab[castle_pos2][0]+desvx2, self.tab[castle_pos2][1]+desvy2, -0.00],[-math.pi,0,math.pi])
		print "11: Sube mas"
	self.mover_baxter('base',[0.537, -0.235 , -0.00],[-math.pi,0,math.pi])
	print "12: Posicion inicial arriba"
	self.en_passant = False
	b.set_en_passant(self.en_passant)
	posep = 0

        self.print_result(b)

    ####################################################################

    def alphabeta(self,depth,alpha,beta,b):

        # We arrived at the end of the search : return the board score
        if(depth==0):
            return b.evaluer()
            # TODO : return quiesce(alpha,beta)

        self.nodes+=1
        self.pv_length[b.ply] = b.ply

        # Do not go too deep
        if(b.ply >= self.MAX_PLY-1):
            return b.evaluer()
            
        # Extensions
        # If king is in check, let's go deeper
        chk=b.in_check(b.side2move) # 'chk' used at the end of func too
        if(chk):
            depth+=1
            
        # TODO
        # sort moves : captures first

        # Generate all moves for the side to move. Those who 
        # let king in check will be processed in domove()
        mList=b.gen_moves_list()

        f=False # flag to know if at least one move will be done
        for i,m in enumerate(mList):
           
            # Do the move 'm'.
            # If it lets king in check, undo it and ignore it
            # remind : a move is defined with (pos1,pos2,promote)
            # i.e. : 'e7e8q' is (12,4,'q')
            if(not b.domove(m[0],m[1],m[2])):
                continue
                
            f=True # a move has passed
            
            score=-self.alphabeta(depth-1,-beta,-alpha,b)

            # Unmake move
            b.undomove()

            if(score>alpha):
                    
                # TODO
                # this move caused a cutoff,
                # should be ordered higher for the next search
                
                if(score>=beta):
                    return beta
                alpha = score

                # Updating the triangular PV-Table
                self.pv[b.ply][b.ply] = m
                j = b.ply + 1
                while(j<self.pv_length[b.ply+1]):
                    self.pv[b.ply][j] = self.pv[b.ply+1][j]
                    self.pv_length[b.ply] = self.pv_length[b.ply + 1]
                    j+=1
                    
        # If no move has been done : it is DRAW or MAT
        if(not f):
            if(chk):
                return -self.INFINITY + b.ply # MAT
            else:
                return 0 # DRAW
                
        # TODO
        # 50 moves rule
            
        return alpha

    ####################################################################

    def print_result(self,b):
        print "print_result"
        "Check if the game is over and print the result"
        
        # Is there at least one legal move left ?
        f=False
        for pos1,pos2,promote in b.gen_moves_list():
            if(b.domove(pos1,pos2,promote)):
		print pos1, pos2                
		b.undomove()
                f=True # yes, a move can be done
                break
        
        # No legal move left, print result
        if(not f):
            if(b.in_check(b.side2move)):
                if(b.side2move=='blanc'):
                    print("0-1 {Black mates}")
                else:
                    print("1-0 {White mates}")
            else:
                print("1/2-1/2 {Stalemate}")
            self.endgame=True
         
        # TODO
        # 3 reps
        # 50 moves rule
            
    ####################################################################

    def clear_pv(self):
        
        "Clear the triangular PV table containing best moves lines"
        
        self.pv=[[0 for x in range(self.MAX_PLY)] for x in range(self.MAX_PLY)]
                
    ####################################################################

    def setboard(self,b,c):
        
        """Set the chessboard to the FEN position given by user with 
        the command line 'setboard ...'.
        'c' in argument is for example :
        'setboard 8/5k2/5P2/8/8/5K2/8/8 w - - 0 0'
        """
        
        cmd=c.split() # split command with spaces
        cmd.pop(0) # drop the word 'setboard' written by user

        # set the FEN position on board
        if(b.setboard(' '.join(cmd))):
            self.endgame=False # success, so no endgame
            
    ####################################################################

    def setDepth(self,c):
        
        """'c' is the user command line, i.e. 'sd [x]'
        to set the search depth.
        """
        
        # Checking the requested value
        cmd=c.split()
        #cmd[0]='sd'
        #cmd[1] should be an integer
        
        try:
            d=int(cmd[1])
        except ValueError:
            print('Depth isn\'t an integer. Please type i.e. : sd 5')
            return
            
        if(d<2 or d>self.MAX_PLY):
            print('Depth must be between 2 and',self.MAX_PLY)
            return
        
        # Things seems to be all right
        self.init_depth=d
        print('Depth set to',d)
        
    ####################################################################

    def perft(self,c,b):
       
        """PERFformance Test :
        This is a debugging function through the move generation tree
        for the current board until depth [x].
        'c' is the command line written by user : perft [x]
        """
        
        # Checking the requested depth
        cmd=c.split()
        #cmd[0]='perft'
        
        try:
            d=int(cmd[1])
        except ValueError:
            print('Please type an integer as depth i.e. : perft 5')
            return
            
        if(d<1 or d>self.MAX_PLY):
            print('Depth must be between 1 and',self.MAX_PLY)
            return
        
        print("Depth\tNodes\tCaptures\tE.p.\tCastles\tPromotions\tChecks\tCheckmates")
        
        time1 = self.get_ms()
        for i in range(1,d+1):
            total=self.perftoption(0,i-1,b)
            print("{}\t{}".format(i,total))
        time2 = self.get_ms()
        timeDiff = round((time2-time1)/1000,2)
        print('Done in',timeDiff,'s')

    def perftoption(self,prof,limit,b):        
        cpt=0

        if(prof>limit):
            return 0

        l=b.gen_moves_list()

        for i,m in enumerate(l):
           
            if(not b.domove(m[0],m[1],m[2])):
                continue

            cpt+=self.perftoption(prof+1,limit,b)

            if(limit==prof):
                cpt+=1

            b.undomove()

        return cpt
        
    ####################################################################
    
    def legalmoves(self,b):
        
        "Show legal moves for side to move"
        
        mList=b.gen_moves_list()
        
        cpt=1
        for m in mList:
            if(not b.domove(m[0],m[1],m[2])):
                continue            
            print('move #',cpt,':',b.caseInt2Str(m[0])+b.caseInt2Str(m[1])+m[2])
            b.undomove()
            cpt+=1
            
    ####################################################################

    def getboard(self,b):
        
        """The user requests the current FEN position
        with the command 'getboard'"""
        
        print(b.getboard())
        
    ####################################################################

    def newgame(self,b):
        
        self.init()
        b.init()
        
    ####################################################################

    def bench(self,b):
        
        """Test to calculate the number of nodes a second.
        The position used is the 17th move of the game :
        Bobby Fischer vs. J. Sherwin, New Jersey State 
        Open Championship, 9/2/1957 :
        1rb2rk1/p4ppp/1p1qp1n1/3n2N1/2pP4/2P3P1/PPQ2PBP/R1B1R1K1 w - - 0 1
        The engine searches to a given depth, 3 following times.
        The kilonodes/s is calculated with the best time.
        """

        oldDepth=self.init_depth
        self.init_depth=4
        timeDiff=[]

        # testing 3 times
        for i in range(3):

            print('Searching to depth',self.init_depth,'...')
            
            if(not b.setboard('1rb2rk1/p4ppp/1p1qp1n1/3n2N1/2pP4/2P3P1/PPQ2PBP/R1B1R1K1 w - - 0 1')):
                print('Could not set board ???!#!')
                return
            
            start_time=self.get_ms()       
            self.search(b)
            stop_time=self.get_ms()
            timeDiff.append(stop_time-start_time)
            print('Time:',timeDiff[i],'ms\n')
            
        if(timeDiff[1] < timeDiff[0]):
            timeDiff[0] = timeDiff[1]
        if(timeDiff[2] < timeDiff[0]):
            timeDiff[0] = timeDiff[2]

        print('Best time:',timeDiff[0],'ms')
        print('Nodes:',self.nodes)
        print('Nodes per second:',round(self.nodes/timeDiff[0],2),'kn/s')
        
        # Restoring changed values
        self.init_depth=oldDepth
        
    ####################################################################

    def undomove(self,b):
        
        "The user requested a 'undomove' in command line"
        
        b.undomove()
        self.endgame=False

    ####################################################################

    def get_ms(self):
        return int(round(time.time() * 1000))

    ####################################################################

    def get_boolc(self):
        return self.boolc

    ####################################################################

    def set_boolc(self, boolcf):
		self.boolc = boolcf

    ####################################################################

    def init(self):
        self.endgame=False    
        self.init_depth=4 # search in fixed depth
        self.nodes=0 # number of nodes
        self.clear_pv()
        
    ####################################################################

        
