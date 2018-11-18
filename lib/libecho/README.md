## Alexa discover process

Send UDP to: 239.255.255.250:1900

Alexa send UDP packet when 'discover devices':

```bash
M-SEARCH * HTTP/1.1
HOST: 239.255.255.250:1900
MAN: "ssdp:discover"
MX: 15
ST: urn:schemas-upnp-org:device:basic:1
```

```bash
M-SEARCH * HTTP/1.1
HOST: 239.255.255.250:1900
MAN: "ssdp:discover"
MX: 15
ST: urn:Belkin:device:**
```


*Device response:*

*ST* must be same as the received packet

```http
HTTP/1.1 200 OK
EXT:
CACHE-CONTROL: max-age=86400
SERVER: Noduino/1.0 UPNP/1.1 OpenLight Controller/1.0
USN: uuid:38323636-4558-4dda-9188-cda0115b5050
ST: urn:Belkin:device:**
LOCATION: http://192.168.1.64:80/setup.xml
```


Then Alexa send HTTP GET request to http://192.168.1.64:80/setup.xml:

```http
GET /setup.xml HTTP/1.1
Host: 192.168.1.81
Accept: */*
```


Device need to implement:

    GET /
    GET /setup.xml


## WEMO

### Turn On

```http
POST /upnp/control/basicevent1 HTTP/1.1
Host: 192.168.1.69
Accept: */*
Content-type: text/xml; charset="utf-8"
SOAPACTION: "urn:Belkin:service:basicevent:1#SetBinaryState"
Content-Length: 299

<?xml version="1.0" encoding="utf-8"?><s:Envelope xmlns:s="http://schemas.xmlsoap.org/soap/envelope/" s:encodingStyle="http://schemas.xmlsoap.org/soap/encoding/"><s:Body><u:SetBinaryState xmlns:u="urn:Belkin:service:basicevent:1"><BinaryState>1</BinaryState></u:SetBinaryState></s:Body></s:Envelope>
```

### Turn Off 

```http
POST /upnp/control/basicevent1 HTTP/1.1
Host: 192.168.1.69
Accept: */*
Content-type: text/xml; charset="utf-8"
SOAPACTION: "urn:Belkin:service:basicevent:1#SetBinaryState"
Content-Length: 299

<?xml version="1.0" encoding="utf-8"?><s:Envelope xmlns:s="http://schemas.xmlsoap.org/soap/envelope/" s:encodingStyle="http://schemas.xmlsoap.org/soap/encoding/"><s:Body><u:SetBinaryState xmlns:u="urn:Belkin:service:basicevent:1"><BinaryState>0</BinaryState></u:SetBinaryState></s:Body></s:Envelope>
```

### Change Name

```http
POST /upnp/control/basicevent1 HTTP/1.1
Host: 192.168.1.69
Accept: */*
Content-type: text/xml; charset="utf-8"
SOAPACTION: "urn:Belkin:service:basicevent:1#ChangeFriendlyName"
Content-Length: 299

<?xml version="1.0" encoding="utf-8"?><s:Envelope xmlns:s="http://schemas.xmlsoap.org/soap/envelope/" s:encodingStyle="http://schemas.xmlsoap.org/soap/encoding/"><s:Body><m:ChangeFriendlyName xmlns:m="urn:Belkin:service:basicevent:1"><FriendlyName>My Switch</FriendlyName></m:ChangeFriendlyName></s:Body></s:Envelope>
```


It is the address and port specified by the UPnP protocol.
Only one such listener is needed since it can send multiple responses,
one for each switch, in response to a search request.

TCP listen on port 49153


## HUE

*setup.xml*

```xml
<root>
<device>
<deviceType>urn:schemas-upnp-org:device:Basic:1</deviceType>
<friendlyName>Philips hue (%s)</friendlyName>
<manufacturer>Royal Philips Electronics</manufacturer>		<------- key
<manufacturerURL>http://www.philips.com</manufacturerURL>
<modelName>Philips hue bridge 2012</modelName>				<------- it's also the key
<modelNumber>929000226503</modelNumber>
<modelURL>http://www.meethue.com</modelURL>
<serialNumber>001788102201</serialNumber>
<UDN>uuid:2f402f80-da50-11e1-9b23-001788102201</UDN>
<presentationURL>index.html</presentationURL>
</device></root>
```

```http
GET /api/h632JhAtwDezSwGk3VKEeFn4snXZUjro9OoEvdur/lights HTTP/1.1
Host: 192.168.1.64
Accept: */*
```

```http
GET /api/h632JhAtwDezSwGk3VKEeFn4snXZUjro9OoEvdur HTTP/1.1
Host: 192.168.1.64
Accept: */*
```


### Dim to 30%

```http
PUT /api/h632JhAtwDezSwGk3VKEeFn4snXZUjro9OoEvdur/lights/0/state HTTP/1.1
Host: 192.168.1.53
Accept: */*
Content-type: application/x-www-form-urlencoded
Content-Length: 21

{"on": true,"bri":76}
```

### bright to 50%

*Request:*

```http
PUT /api/h632JhAtwDezSwGk3VKEeFn4snXZUjro9OoEvdur/lights/0/state HTTP/1.1
Host: 192.168.1.53
Accept: */*
Content-type: application/x-www-form-urlencoded
Content-Length: 22

{"on": true,"bri":127}
```


*Response:*

```http
HTTP/1.1 200 OK
Cache-Control: no-store, no-cache, must-revalidate, post-check=0, pre-check=0
Content-type: application/json

[{"success": {"on": true}}, {"success": {"bri": 255}}]
```


### Turn on

*Request:*

```http
PUT /api/h632JhAtwDezSwGk3VKEeFn4snXZUjro9OoEvdur/lights/0/state HTTP/1.1
Host: 192.168.1.53
Accept: */*
Content-type: application/x-www-form-urlencoded
Content-Length: 12

{"on": true}
```


```http
PUT /api/h632JhAtwDezSwGk3VKEeFn4snXZUjro9OoEvdur/lights/0/state HTTP/1.1
Host: 192.168.1.53
Accept: */*
Content-type: application/x-www-form-urlencoded
Content-Length: 13

{"on": false}
```
