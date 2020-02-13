import requests
import json
import datetime
import os

from credentials import *
from pydrive.auth import GoogleAuth
from pydrive.drive import GoogleDrive


# Google Authentication for Drive
gauth = GoogleAuth()
gauth.LocalWebserverAuth()
drive = GoogleDrive(gauth)

# Authentication parameters
headers = {
			'AccountKey' : access_key,
			'accept' : 'application/json'
}


def fetch_all(path):	

	uri = 'http://datamall2.mytransport.sg/'
	results = []

	while True:
		new_results = requests.get(
			uri + path,
			headers = headers,
			params = {'$skip': len(results)} # 50 records/call
		).json()['value']

		if new_results == []:
			break
		else:
			results += new_results

	return results


def get_folder_id(folder_name):
    
    # Get all folders and files in your Google Drive
    fileList = drive.ListFile({'q': "'root' in parents and trashed=false"}).GetList()
    
    for file in fileList:
        if(file['title'] == folder_name):
            return file['id']


if __name__=="__main__":

	# Upload it into Google drive with name as: Date_Day_Time
	x = datetime.datetime.now()
	file_name = x.strftime("%c") + ".json"

	path = '/ltaodataservice/TrafficSpeedBandsv2'
	traffic_speed_bands = fetch_all(path) #58,780    

	with open(file_name, "w") as f:
		f.write(json.dumps(traffic_speed_bands, sort_keys=True, indent=4, ensure_ascii=False))

	folder_id = get_folder_id("Traffic Speed Band Dataset")
	file1 = drive.CreateFile({"mimeType": "text/plain", "parents": [{"kind": "drive#fileLink", "id": folder_id}]})
	file1.SetContentFile(file_name)
	file1.Upload()
	print('Created file %s with mimeType %s' % (file1['title'], file1['mimeType']))

	# Delete local copy of file to save space
	if os.path.exists(file_name):
		os.remove(file_name)
