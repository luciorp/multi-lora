# -*- coding: utf-8 -*-
"""
Created on Tue Apr 9 13:58:52 2019

@author: lucio
"""

import subprocess as sbp
import pandas as pd
import math
import node

topology = pd.read_csv("C:/Users/lucio/Downloads/simulator/topology.csv") 

def inside_radius(xcoord, ycoord, dfxcoord, dfycoord, radius):
    
    '''
    Parameters:
        xcoord: PL in the equation # http://www.semtech.com/images/datasheet/sx1276.pdf.  PHY Payload size in byte (= MAC Payload + 5)
        ycoord: SF (12 to 7)
        dfxcoord: if enable_auto_ldro is disabled, LDRO is disable by default,
        dfycoord: when enable_eh is set to False, IH in the fomula is going to be 1.
        radius: when enable_crc is set to False, CRC in the equation is going to be 0.
    Return:
        True or false
    '''
    if (xcoord != None and ycoord != None and dfxcoord != None and dfycoord != None):
        lat1 = float(xcoord)
        lon1 = float(ycoord)
        lat2 = float(dfxcoord)
        lon2 = float(dfycoord)
        if ((math.dist([lat1,lon1], [lat2, lon2]) <= radius) and ((xcoord != dfxcoord) or (ycoord != dfycoord))):
            return  1
        else:
            return  0


def calc_time_on_air(n_size, n_sf, n_bw=125, enable_auto_ldro=True, enable_ldro=False,
            enable_eh=True, enable_crc=True, n_cr=1, n_preamble=8):
    '''
    Parameters:
        n_size:
            PL in the equation # http://www.semtech.com/images/datasheet/sx1276.pdf.  PHY Payload size in byte (= MAC Payload + 5)
        n_sf: SF (12 to 7)
        n_bw: Bandwidth in kHz.  default is 125 kHz for AU915.
        enable_auto_ldro
            flag whether the auto Low Data Rate Optimization is enabled or not.
            default is True.
        enable_ldro:
            if enable_auto_ldro is disabled, LDRO is disable by default,
            which means that DE in the equationis going to be 0.
            When enable_ldro is set to True, DE is going to be 1.
            SX1276 datasheet reuiqres to enable LDRO
            when the symbol duration exceeds 16ms.
        enable_eh:
            when enable_eh is set to False, IH in the fomula is going to be 1.
            default is True, which means IH is 0.
        enable_crc:
            when enable_crc is set to False, CRC in the equation is going to be 0.
        n_cr:
            CR in the fomula, should be from 1 to 4.
        n_preamble:
            The preamble length in bit.
            default is 8 in AU915.
    Return:
        t_packet: time on air in *milisecond*.
    '''
    r_sym = (n_bw*1000.) / math.pow(2,n_sf)
    t_sym = 1000. / r_sym
    t_preamble = (n_preamble + 4.25) * t_sym
    v_DE = 0
    if enable_auto_ldro:
        if t_sym > 16:
            v_DE = 1
    elif enable_ldro:
        v_DE = 1
    v_IH = 0
    if not enable_eh:
        v_IH = 1
    v_CRC = 1
    if enable_crc == False:
        v_CRC = 0
    a = 8.*n_size - 4.*n_sf + 28 + 16*v_CRC - 20.*v_IH
    b = 4.*(n_sf-2.*v_DE)
    n_payload = 8 + max(math.ceil(a/b)*(n_cr+4), 0)
    t_payload = n_payload * t_sym
    t_packet = t_preamble+ t_payload

    return round(t_packet, 2)




topology['inside_radius'] = topology.apply(lambda row : inside_radius(0,0,row['Xcoord'],row['Ycoord'], 4), axis = 1)

toa = calc_time_on_air(18, 7)

print(topology)

print(toa)



node1 = node(["C:/Users/lucio/Downloads/teste_stdin.exe"])
node1.sendArray("teste22")
