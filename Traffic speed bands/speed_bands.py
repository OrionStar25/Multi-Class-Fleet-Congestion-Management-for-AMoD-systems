import json
import urllib.request, urllib.parse, urllib.error
from urllib.parse import urlparse
import httplib2 as http
from credentials import *


if __name__=="__main__":

	#Authentication parameters
	headers = { 'AccountKey' : access_key,
	'accept' : 'application/json'} #this is by default

	#API parameters and specify type of API call
	uri = 'http://datamall2.mytransport.sg/' #Resource URL
	path = '/ltaodataservice/TrafficSpeedBandsv2'
	skip = '?$skip='
	method = 'GET'
	body = ''

	#Get handle to http
	h = http.Http()

	#Build query string
	target = urlparse(uri + path)
	i = 0

	while i < 2001:
		print(target.geturl())

		#Obtain results
		response, content = h.request(
		target.geturl(),
		method,
		body,
		headers)

		#Parse JSON to print
		jsonObj = json.loads(content)
		# print(json.dumps(jsonObj, sort_keys=True, indent=4))

		#Save result to file
		with open("speed_bands.json","a+") as outfile:
			#Saving jsonObj["d"]
			json.dump(jsonObj, outfile, sort_keys=True, indent=4,
			ensure_ascii=False)

		#retrieve subsequent data
		i += 500
		target = urlparse(uri + path + skip + str(i))