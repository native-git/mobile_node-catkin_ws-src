import os
# We'll render HTML templates and access data sent by POST
# using the request object from flask. Redirect and url_for
# will be used to redirect the user once the upload is done
# and send_from_directory will help us to send/show on the
# browser the file that the user just uploaded
from flask import Flask, render_template, request, redirect, url_for, send_from_directory
from werkzeug import secure_filename

import socket
import time
import linecache

global server, location_port, target_port, host_port
lineNum = 1

server = '192.168.207.201'
host_port = 5000
location_port = 10001
target_port = 10002


# Initialize the Flask application
app = Flask(__name__)

# This is the path to the upload directory
app.config['UPLOAD_FOLDER'] = 'uploads/'
# These are the extension that we are accepting to be uploaded
app.config['ALLOWED_EXTENSIONS'] = set(['txt', 'pdf', 'png', 'jpg', 'jpeg', 'gif'])

# For a given file, return whether it's an allowed type or not
def allowed_file(filename):
	return '.' in filename and \
		filename.rsplit('.', 1)[1] in app.config['ALLOWED_EXTENSIONS']

# This route will show a form to perform an AJAX request
# jQuery is loaded to execute the request and update the
# value of the operation
@app.route('/')
def index():
	return render_template('index.html')


# Route that will process the file upload
@app.route('/upload', methods=['POST'])
def upload():
	# Get the name of the uploaded file
	file = request.files['file']
	# Check if the file is one of the allowed types/extensions
	if file and allowed_file(file.filename):
		# Make the filename safe, remove unsupported chars
		filename = secure_filename(file.filename)
		# Move the file form the temporal folder to
		# the upload folder we setup
		file.save(os.path.join(app.config['UPLOAD_FOLDER'], filename))
		# Redirect the user to the uploaded_file route, which
		# will basicaly show on the browser the uploaded file
		return redirect(url_for('uploaded_file',
								filename=filename))

# This route is expecting a parameter containing the name
# of a file. Then it will locate that file on the upload
# directory and show it on the browser, so if the user uploads
# an image, that image is going to be show after the upload
@app.route('/uploads/<filename>')
def uploaded_file(filename):
	#return send_from_directory(app.config['UPLOAD_FOLDER'], filename)
	global lineNum
	file123 = send_from_directory(app.config['UPLOAD_FOLDER'], filename)
	line = open(file123).readlines()[lineNum]
	if(line):
		lineNum = lineNum+1
		return redirect(url_for('go_to', x=line, y=line, r=line))
	else:
		return send_from_directory(app.config['UPLOAD_FOLDER'], filename)
	

@app.route('/go_to')
def go_to():
	
	received = False

	x = request.args.get('x', type=float)
	y = request.args.get('y', type=float)
	yaw = request.args.get('r', type=float)

	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((server,target_port))

	# Receive the auto-replied position information
	msg = str(x)+","+str(y)+","+str(0.0)+":"+str(0.0)+","+str(0.0)+","+str(yaw)+"\n"

	while not received:

		s.send(msg)
		response = s.recv(20)

		try:
			response = response.strip('\n')
			if response == 'BAD':
				break
			elif response == 'OK':
				s.close()
				received = True

		except IndexError:
			return "Please try again, bad connection to target_server"
			continue

	s.close()
	time.sleep(30)
	return redirect(url_for('uploaded_file', filename='test.txt'))
	#return 'Setting New Target-<br/>X: ' + str(x) + " Y: " + str(y) + " THETA: " + str(yaw)

if __name__ == '__main__':
	app.run(server,host_port)
