Autor: José Santos email:jmpsantos@ua.pt
Co-Autor: João Viegas email:jpviegas@ua.pt

DESCRIÇÃO BREVE DO SCRIPT:
Este script tem como funcionalidade:
	1.Ir buscar dados (Endereço MAC,RSSI) a um Webserver criado pelo Arduino;
	2.Tratar os dados de modo a obtermos a partir da info dos vários Arduinos(3 unidades) obtermos a posição;
	3.Guardar os valores das posiçoes de cada utilizador em duas tabelas na base de dados (1 tabela são os valores atuais e na outra é o historico)

ESPECIFICAÇÕES DE TESTE:
Este programa irá correr no pc/rasp (não testamos na rasp somente num pc), este pc estava a correr o windows 10 e o código corria no Visual Studio 2019;
Para correr o código será também preciso ter o mySQL comunity server e a base de dados deverá ter as seguintes tabelas:

-location name varchar(20) primary key,x DECIMAL(3,2),y DECIMAL(3,2),t int
-rawdata1 name varchar(20) primary key,RSSI int,t int
-rawdata2 name varchar(20) primary key,RSSI int,t int
-rawdata3 name varchar(20) primary key,RSSI int,t int
-users	name varchar(20) primary key
-historico name varchar(20),x int,y int,t int
-saveflag flag varchar(20) primary key,f int
-starterips arduino int primary key,positionx varchar(10),positiony varchar(10)
-starterpos arduino int primary key,ip varchar(30)

Quanto a bibliotecas são necessárias as seguintes:

import math
import time 
import random
from scipy.optimize import minimize
import random 
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import itertools
import numpy as np
import requests
from bs4 import BeautifulSoup
import re
from requests.exceptions import HTTPError

DESCRIÇÃO DO CÓDIGO:
O código foi dividido em 10 separações:

1:Initialize BD
 -Inicializa a base de dados (MySQL);

2:Hashing User ID
 -Função para dar Hash ao endereço MAC mas que nao a chegamos a usar pois o dicionario de Hashing mudava a cada compilação;

3:Inicializar algumas Variaveis
 -Iniciamos a timestamp e imprimimos (nesta versão não usamos este valor em nenhuma ocasião);

4:Configuração Inicial
 -Vamos a base de dados da tabela saveflag verificar se a configuração das posiçoes iniciais foram feitas (1-True 0- False);
 -O código fica preso aqui se a configuração nao for feita na interface gráfica(que muda a flag para 1 na BD quando se grava);
 -Esta posição inicial é essencial para o cálculo da posição;

5:Adquirir dados do webserver (Modulo feito por co-autor)
 -Vamos nos ligar em primeiro ao webserver do arduino 1 com um IP introduzido aqui manualmente (Ainda nao conseguimos tornar o IP estatico de cada arduino);
 -O código fica preso até se ligar ao webserver1, depois guarda todos os dados numa string 'info' e envia para a tabela rawdata 1 o endereço MAC e o valor de RSSI e para a tabela Users o endereço MAC;
 -Após se ligar ao webserver1 passa a tentar ligar-se ao webserver 2 e 3 e envia os dados agora para a tabela rawdata2 e rawdata3 respetivamente (e para a tabela Users também)

6:Conversão de RSSI para Metros
 -Aqui criamos uma função que pega no valor de RSSI e converte para metro;
 -Temos duas versões, a comentada é uma versão inicial que tentamos implementar de outros autores, a segunda que é a que estamos a usar atualmente foi criada por um membro do grupo (João Domingues) que definiu que mais se aproximava aos resultados pretendidos;
 -Nesta versão também temos que para valores de RSSI iguais ou maiores que |-120| (valor a qual o erro era bastante significativo >10%) devolve '-1' 

7:Ciclo de utilizadores
 -Esta secção serve para percorremos a tabela Users e a cada endereço MAC vamos a cada tabela dos dados raw (rawdata#) e retirar o valor de RSSI obtido em cada arduino;

8:Verificar BLE's FUNCIONAIS
 -Aqui fazemos a seleção dos arduinos que deram valores corretos de RSSI (diferente de -1 como vimos no passo 6.)


9:Algoritmo para o cálculo da Posição
 -Nesta parte fazemos agora o calculo da posição de cada User, este cálculo é feito pela aproximação inicial da posição da pessoa (no meio do espaço quando nao temos um valor de posição anterior ou quando temos tomamos esse mesmo valor) e minimizamos a soma dos erros que temos dessa aproximação e do valor obtido pelo RSSI;

10:TO DATABASE
 -Por fim enviamos as posiçoes para a tabela location e historico

NOTAS FINAIS:
 -As tabelas estão preparadas para receber valores com casas decimais mas no código estamos a tratas os valores como Int por isso na base de dados as posiçoes aparecem sem casas decimais;

