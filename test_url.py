import http.client

connection = http.client.HTTPSConnection("localhost:8088")
connection.request("GET", "/")
response = connection.getresponse()
print("/nStatus: {} and reason: {}".format(response.status, response.reason))

connection.close()
