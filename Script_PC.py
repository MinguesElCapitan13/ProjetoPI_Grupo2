#Version 7 Author: Jose Santos jmpsantos@ua.pt
#        Co-Autho: João Viegas jpviegas@ua.pt        

import math
import time 
import random
from scipy.optimize import minimize
import random 
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import itertools
import numpy as np

#Imports do webserver
import requests
from bs4 import BeautifulSoup     # instala estas duas libs: pip install request (e dps o BeautifulSoup)
import re
from requests.exceptions import HTTPError
global k
k=0

while True:
   
   
    #-------------- 1:Initialize BD ---------------#
    import mysql.connector
    mydb = mysql.connector.connect(
        host="localhost",
        user="root",
        password="Kasuno196522!",
        database="testpi"
        )
    mydb.autocommit = True
    mycursor = mydb.cursor()



    #------------- 2:Hashing User ID -------------#
    #Está a funcionar mas como para debug o valor muda a cada run não uso por agora

    #UserID='1'
    #HUserID=hash(UserID)
    #print(HUserID)

    #the hash of a value only needs to be the same for one run of Python. 
    #In Python 3.3 they will in fact change for every new run of Python
    #harder to guess what hash value a certain string will have -> more secure



    ##------------- 3:Inicializar algumas Variaveis -------------##

    #Variveis que é necessário receber: User,RSSI,timestamp
    timestamp=time.time()  #time in seconds since the epoch(number of seconds that have elapsed since January 1, 1970)
    print(timestamp)
    
    ##------------- 4:Configuração Inicial -------------##

    mycursor.execute("SELECT f from saveflag")
    flag=mycursor.fetchall()


    while flag[0][0]==0 :
            mycursor.execute("SELECT f from saveflag")
            flag=mycursor.fetchall()
            print('Please make the initial configuration on the IG')
            time.sleep(1)

    print('Configuração feita')
            

    mycursor.execute("SELECT positionx,positiony from starterpos")
    starterpos=mycursor.fetchall()
    BLE_locations= [ (int(starterpos[0][0]),int(starterpos[0][1])), (int(starterpos[1][0]),int(starterpos[1][1])),(int(starterpos[2][0]),int(starterpos[2][1]))] #Posição dos BLE's


    ##------------- 5:Adquirir dados do webserver -------------##

    i=0
    url0 = 'http://192.168.1.104/'  #Arduino na posição 1
    url1 = 'http://192.168.1.103/'  #Arduino na posição 2
    url2 = 'http://192.168.1.101/'  #Arduino na posição 3
    while i< 3:
        linha=0
        while True:
            try:
                if i == 0 :
                    response = requests.get(url0, timeout=2)
                    # If the response was successful, no Exception will be raised
                    response.raise_for_status()
                if i == 1:
                    response = requests.get(url1, timeout=2)
                    # If the response was successful, no Exception will be raised
                    response.raise_for_status()
                if i == 2 :
                    response = requests.get(url2, timeout=2)
                    # If the response was successful, no Exception will be raised
                    response.raise_for_status()

            except HTTPError as http_err:
                    #print(f'HTTP error occurred: {http_err}')  
                    print('\n')

            except Exception as err:
                    #print(f'Other error occurred: {err}')  
                    print('WebServer Down\n')
               
            else:
                    print('Success!')
                    soup = BeautifulSoup(response.text, 'html.parser')
                    info = re.findall(r'MAC: ((?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}); RSSI: (-?\d+)', soup.get_text())
                    print(info)
                    break

        while linha<len(info):  # tamanho do string[i][] Pode estar errado
            #Adicina a cada tabela da base de dados (uma para cada arduino) os seus valores
            mycursor.execute("insert into rawdata"+str(int(i+1))+" values('"+str(info[linha][0])+"',"+str(int(info[linha][1]))+","+str(int(0))+") on duplicate key update RSSI = "+str(int(info[linha][1]))+" , t = "+str(int(0))+";")
            mydb.commit()
            #Adiciona a tabela users todos os utilizadores
            mycursor.execute("insert IGNORE into users values('"+str(info[linha][0])+"');")    #ERROR on duplicate
            mydb.commit()

            #mycursor.execute("insert into fullrawdata"+str(int(i+1))+" values('"+str(info[linha][0])+"',"+str(int(info[linha][1]))+")")    #New method in development
            mydb.commit()

            linha=linha+1
        #FIM DO WHILE DE LINHA A LINHA
        i=i+1
    #FIM DO WHILE DOS URLS

    ##------------- 6:Conversão de RSSI para Metros -------------##

   # def rssiToMetters(rssi):				#Função que traduz RSSI para metros
       # if rssi == 0: distance=-1.0			#se um BLE estiver estragado (não está FUNCIONAL)
       # txPower=-82                         #Calibrado para quando o RSSI deveria estar 1m
       # ratio= rssi*1.0/txPower

       # if (ratio < 1.0): distance= math.pow(ratio,10)
       # else: distance= (0.89976)*math.pow(ratio,7.7095) + 0.111  
       # print("%.2f" % distance,"metros")
       # return distance;

    def rssiToMetters(rssi):				 #Função alternativa (autor:João Mingues) que traduz RSSI para metros
        if rssi < -120 : distance=-1.0	     #se um BLE estiver offline (não está FUNCIONAL)
        txPower=-105                         #Calibrado para quando o RSSI deveria estar 1m 
        ratio= 2
        aux=(txPower-rssi)/(ratio)
        distance=math.pow(10,aux)

        return distance;

    def great_circle_distance(x1, y1, x2, y2):   #Função que calcula a distancia entre pontos
        return math.sqrt(math.pow(x1-x2,2)+math.pow(y1-y2,2))

    ##------------- 7:Ciclo de utilizadores -------------##
    mycursor.execute("SELECT * from users")
    names=mycursor.fetchall()
    name=0
    while name<len(names):                  #Ir de utilizador a utilizador para calcular a posição atual
        x=[]
        print(names[name][0])

        mycursor.execute("SELECT rssi from rawdata1 WHERE name = '"+str(names[name][0])+"'")
        temp=mycursor.fetchall()
        if temp==[]:                        #Se neste Arduino nao detetar o user envia um 0 para nao contar com este valor
            x.append(0)
        else:
            x.append(rssiToMetters(temp[0][0]))   #Adiciona os valores das distancias a um array x

        mycursor.execute("SELECT rssi from rawdata2 WHERE name = '"+str(names[name][0])+"'")
        temp=mycursor.fetchall()
        if temp==[]:                        #Se neste Arduino nao detetar o user envia um 0 para nao contar com este valor
            x.append(0)
        else:
            x.append(rssiToMetters(temp[0][0]))   #Adiciona os valores das distancias a um array x

        mycursor.execute("SELECT rssi from rawdata3 WHERE name = '"+str(names[name][0])+"'")
        temp=mycursor.fetchall()
        if temp==[]:                        #Se neste Arduino nao detetar o user envia um 0 para nao contar com este valor
            x.append(0)
        else:
            x.append(rssiToMetters(temp[0][0]))   #Adiciona os valores das distancias a um array x


    #x.append(rssiToMetters(55)*100)   #Adiciona os valores manualmente HARDWIRED
    #x.append(rssiToMetters(55)*100)
    #x.append(rssiToMetters(55)*100)
   

    ##------------- 8:Verificar BLE's FUNCIONAIS -------------##

        locations=[]				#Array das localizações com BLE's FUNCIONAIS
        distances=[]				#Array com os raios de cada BLE FUNCIONAL

        for i in range(0,3):							#Ciclo For i=0:1:2   Neste caso que só temos 3 arduinos
            if x[i] > 0:								#Confirma se o BLE está FUNCINAL (x[i]>0)
                locations.append(BLE_locations[i])		#Adiciona a localização do BLE funcional ao array
                distances.append(x[i])					#Adiciona a distancia do BLE funcional ao array



    ##------------- 9:Algoritmo para o cálculo da Posição -------------##

    # Mean Square Error								#Função para calcular o erro medio minimo para estimar a posição atual
    #Escolha do Valor inicial, se o utilizador já tiver uma posição obtida na BD usa essa como inicial, senao usamos o centro da loja
        mycursor.execute("SELECT EXISTS(SELECT * FROM location WHERE name = '"+str(names[name][0])+"')")
        bol=mycursor.fetchall() 
    #print(bol) Usado para debug
        if bol[0][0]>0:
            mycursor.execute("SELECT x,y from location WHERE name='"+str(names[name][0])+"'")
            x_init,y_init = mycursor.fetchall()[0]
            initial_location=(x_init,y_init)
        else:
            initial_location=(50,50)						



        def mse(x, locations, distances):            
            mse = 0.0
            for location, distance in zip(locations, distances):
                distance_calculated = great_circle_distance(x[0], x[1], location[0], location[1])
                mse += math.pow(distance_calculated - distance, 2.0)
            return mse / len(distances)



        result = minimize(                                       #Funcão que encontra a posição em que apresenta menor erro
	        mse,                         # The error function
	        initial_location,            # The initial guess
	        args=(locations, distances), # Additional parameters for mse
	        method='L-BFGS-B',           # The optimisation algorithm
	        options={
		        'ftol':1e-5,         # Tolerance
		        'maxiter': 1e+7      # Maximum iterations
	        })
        location = result.x

        print("("+str(int(location[0]))+","+str(int(location[1]))+")")




    ##--------------------- 10:TO DATABASE--------------------------##

        mycursor.execute("insert into location values('"+str(names[name][0])+"',"+str(int(location[0]))+","+str(int(location[1]))+","+str(int(timestamp))+") on duplicate key update x = "+str(int(location[0]))+" , y = "+str(int(location[1]))+" , t = "+str(int(timestamp))+";")
        mydb.commit()

        mycursor.execute("insert into historico values('"+str(names[name][0])+"',"+str(int(location[0]))+","+str(int(location[1]))+","+str(int(timestamp))+")") 
        mydb.commit()

        name=name+1         #Passa a fazer o calculo da posição para o proximo utilizador

