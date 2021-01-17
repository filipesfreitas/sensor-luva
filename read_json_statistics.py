import json
import numpy as np
import matplotlib.pyplot as plt

deslocamento = 10
tam_janela = 10
media_movel = np.array([])
data_array = np.array([])
with open('packet_measure.json') as file:
	data = json.load(file)
	
	for i in range(0,int(len(data)/tam_janela)):
		data_array = np.append(data_array,[float(data[i]["_source"]["layers"]["frame"]["frame.time_delta"])])
		media_movel = np.append(media_movel,[sum([float(data[a]["_source"]["layers"]["frame"]["frame.time_delta"]) for a in range(i*tam_janela,i*tam_janela+deslocamento)])/tam_janela] )
				
np.amin(media_movel)
np.amax(media_movel)
np.percentile(media_movel,25,interpolation='nearest')
np.median(media_movel)
np.average(media_movel)
np.mean(media_movel)
np.std(media_movel)
np.var(media_movel)


np.percentile(data_array,25,interpolation='nearest')

np.mean(data_array)
np.std(data_array)
np.var(data_array)
