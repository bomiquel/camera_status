#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Sep  2 18:38:14 2021

@author: uib
"""

#%%
import pandas as pd  

#%%

data = {'Name':['Renault', 'Duster', 'Maruti', 'Honda City'], 'Ratings':[9.0, 8.0, 5.0, 3.0]} 
df = pd.DataFrame(data)  
print(df)   

#%%

df = df.drop(labels=range(0, len(df)), axis=0)