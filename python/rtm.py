from omniORB import CORBA, any, cdrUnmarshal, cdrMarshal
import CosNaming

import RTC, OpenRTM, SDOPackage, RTM
from OpenRTM import CdrData, OutPortCdr, InPortCdr
from RTC import *

import sys
import string, math, socket
import os
import time

##
# \brief root naming context
#
rootnc = None

##
# \brief wrapper class of RT component
#
class RTcomponent:
	##
	# \brief constructor
	# \param self this object
	# \param ref IOR of RT component
	#
	def __init__(self, ref):
		self.ref = ref
		self.owned_ecs = ref.get_owned_contexts()
		self.ec = self.owned_ecs[0]
		self.ports = {}
		ports = self.ref.get_ports()
		for p in ports:
			prof = p.get_port_profile()
			name = prof.name.split('.')[1]
			self.ports[name] = p
	
	##
	# \brief get IOR of port
	# \param self this object
	# \param name name of the port
	# \return IOR of the port
	def port(self, name):
		try:
			p = self.ports[unicode(name)]
		except KeyError:
			p = findPort(self.ref, name)
			self.ports[unicode(name)] = p
		return p

	##
	# \brief get IOR of service
	# \param self this object
	# \param name name of service
	# \return IOR of the service
	def service(self, name):
		return findService(self.ref, name)

	##
	# \brief update default configuration set
	# \param self this object
	# \param nvlist list of pairs of name and value
	def setConfiguration(self, nvlist):
		setConfiguration(self.ref, nvlist)

        ##
	# \brief update value of the default configuration set
	# \param self this object	
	# \param name name of the property
	# \param value new value of the property
	def setProperty(self, name, value):
		self.setConfiguration([[name, value]])

        ##
	# \brief get value of the property in the default configuration set
	# \param self this object	
	# \param name name of the property
        # \return value of the property or None if the property is not found
	def getProperty(self, name):
		cfg = self.ref.get_configuration()
		cfgsets = cfg.get_configuration_sets()
		if len(cfgsets) == 0:
			print "configuration set is not found"
			return None
		cfgset = cfgsets[0]
		for d in cfgset.configuration_data:
			if d.name == name:
				return any.from_any(d.value)
		return None		

	##
	# \brief activate this component
	# \param self this object
	# \param ec execution context used to activate this component
	def start(self, ec=None):
		if ec == None:
			ec = self.ec
		if ec != None:
			ec.activate_component(self.ref)
			while self.isInactive():
				time.sleep(0.01)

	##
	# \brief deactivate this component
	# \param self this object
	# \param ec execution context used to deactivate this component
	def stop(self, ec=None):
		if ec == None:
			ec = self.ec
		if ec != None:
			ec.deactivate_component(self.ref)
			while self.isActive():
				time.sleep(0.01)

	##
	# \brief get life cycle state of the main execution context
	# \param self this object
	# \param ec execution context from which life cycle state is obtained
        # \return one of LifeCycleState value or None if the main execution context is not set
	def getLifeCycleState(self, ec=None):
		if ec == None:
			ec = self.ec
		if ec != None:
			return ec.get_component_state(self.ref)
		else:
			return None

	##
	# \brief check the main execution context is active or not
        # \retval 1 this component is active
        # \retval 0 this component is not active
        def isActive(self):
		return RTC.ACTIVE_STATE == self.getLifeCycleState()

	##
	# \brief check the main execution context is inactive or not
        # \retval 1 this component is inactive
        # \retval 0 this component is not inactive
        def isInactive(self):
		return RTC.INACTIVE_STATE == self.getLifeCycleState()

	##
	# \brief get instance name
	# \return instance name
	def name(self):
		cprof = self.ref.get_component_profile()
		return cprof.instance_name

##
# \brief wrapper class of RTCmanager
#
class RTCmanager:
	##
	# \brief constructor
	# \param self this object
	# \param ref IOR of RTCmanager
	#
	def __init__(self, ref):
		self.ref = ref
		uname = os.uname()[0]
		if uname == "Darwin":
			self.soext = ".dylib"
		else:
			self.soext = ".so"
	
	##
	# \brief load RT component factory
	# \param self this object
	# \param basename common part of path of the shared library and the initialize function. path is generated by basename+".so" and the initialize function is generated by basename+"Init".
	#
	def load(self, basename):
		path = basename+self.soext
		initfunc = basename+"Init"
		try:
			self.ref.load_module(path, initfunc)
		except:
			print "failed to load",path

	##
	# \brief create an instance of RT component
	# \param self this object
	# \param module name of RT component factory
        # \param name name of RT component instance
	# \return an object of RTcomponent
	def create(self, module,name=None):
		if name != None:
			rtc = findRTC(name)
			if rtc != None:
				print 'RTC named "',name,'" already exists.'
				return rtc
		args = module
		if name != None:
			args += '?instance_name=' + name
		ref = self.ref.create_component(args)
		if ref == None:
			return None
		else:
			return RTcomponent(ref)

	##
	# \brief get list of factory names
        # \return list of factory names
	def get_factory_names(self):
		fs = []
		fps = self.ref.get_factory_profiles()
		for afp in fps:
			for p in afp.properties:
				if p.name == "implementation_id":
					fs.append(any.from_any(p.value))
		return fs

	##
	# \brief get list of components
        # \return list of components
	def get_components(self):
		cs = []
		crefs = self.ref.get_components()
		for cref in crefs:
			c = RTcomponent(cref)
			cs.append(c)
		return cs	

	##
	# \brief restart Manager
	def restart(self):
		self.ref.shutdown()
		time.sleep(1)
##
# \brief unbind an object reference 
# \param name name of the object
# \param kind kind of the object
#
def unbindObject(name, kind):
	nc = NameComponent(name, kind)
	path = [nc]
	rootnc.unbind(path)
	return None

##
# \brief initialize ORB 
#
def initCORBA():
	global rootnc, orb
	orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)

	nameserver = orb.resolve_initial_references("NameService");
	rootnc = nameserver._narrow(CosNaming.NamingContext)
	return None

##
# \brief get root naming context 
# \param corbaloc location of NamingService
# \return root naming context
#
def getRootNamingContext(corbaloc):
	props = System.getProperties()
	
	args = ["-ORBInitRef", corbaloc]
	orb = ORB.init(args, props)

	nameserver = orb.resolve_initial_references("NameService");
	return NamingContextHelper.narrow(nameserver);

##
# \brief get IOR of the object
# \param name name of the object
# \param kind kind of the object
# \param rnc root naming context. If it is not specified, global variable rootnc is used
# \return IOR of the object
# 
def findObject(name, kind, rnc=None):
	nc = CosNaming.NameComponent(name,kind)
	path = [nc]
	if not rnc:
		rnc = rootnc
	return rnc.resolve(path)

##
# \brief get RTCmanager
# \param hostname hostname where rtcd is running
# \param rnc root naming context. If it is not specified, global variable rootnc is used
# \return an object of RTCmanager
#
def findRTCmanager(hostname=socket.gethostname(), rnc=None):
	try:
		cxt = findObject(hostname, "host_cxt", rnc)
		obj = findObject("manager","mgr",cxt)
		return RTCmanager(obj._narrow(RTM.Manager))
	except:
		print "exception in findRTCmanager("+hostname+")"

##
# \brief get RT component
# \param name name of the RT component
# \param rnc root naming context. If it is not specified, global variable rootnc is used
# \return an object of RTcomponent
#
def findRTC(name, rnc=None):
	try:
		obj = findObject(name, "rtc", rnc)
		rtc = RTcomponent(obj._narrow(RTC.RTObject))
		cxts = rtc.ref.get_participating_contexts()
		if len(cxts) > 0:
			rtc.ec = cxts[0]
		return rtc
	except:
		return None

##
# \brief get a port of RT component
# \param rtc an object of RTcomponent
# \param name name of the port
# \return IOR of the port if the port is found, None otherwise
#
def findPort(rtc, name):
	ports = rtc.get_ports()
	cprof = rtc.get_component_profile()
	portname = cprof.instance_name+"."+name
	for p in ports:
		prof = p.get_port_profile()
		if prof.name == portname:
			return p
	return None 

##
# \brief set up execution context of the first RTC so that RTCs are executed sequentially
# \param rtcs sequence of RTCs
#
def serializeComponents(rtcs):
	if len(rtcs) < 2:
		return
	ec = rtcs[0].ec
	for rtc in rtcs[1:]:
		if ec != rtc.ec: 
			rtc.ec.stop()
			if ec.add_component(rtc.ref) == RTC.RTC_OK:
				rtc.ec = ec
			else:
				print 'error in add_component()'
		else:
			print 'already serialized'

##
# \brief check two ports are connected or not
# \retval True connected
# \retval False not connected
def isConnected(outP, inP):
	op = outP.get_port_profile()
	for con_prof in op.connector_profiles:
		ports = con_prof.ports
		if len(ports) == 2 and outP._is_equivalent(ports[0]) and inP._is_equivalent(ports[1]):
			return True
	return False
	
##
# \brief disconnect ports
# \param outP IOR of outPort
# \param inP IOR of inPort
def disconnectPorts(outP, inP):
	op = outP.get_port_profile()
	iname = inP.get_port_profile().name
	for con_prof in op.connector_profiles:
		ports = con_prof.ports
		pname = ports[1].get_port_profile().name
		if len(ports) == 2 and pname == iname:
			outP.disconnect(con_prof.connector_id)
	return

##
# \brief get data type of a port
# \param port IOR of port
# \return data type
def dataTypeOfPort(port):
	prof = port.get_port_profile()
	prop = prof.properties
	for p in prop:
		if p.name == "dataport.data_type":
			return any.from_any(p.value)
	return None

##
# \brief connect ports
# \param outP IOR of outPort 
# \param inPs an IOR or a list of IORs of inPort
# \param subscription subscription type. "flush", "new" or "periodic"
# \param dataflow dataflow type. "Push" or "Pull"
# \param bufferlength length of data buffer
#
def connectPorts(outP, inPs, subscription="flush", dataflow="Push", bufferlength=1, rate=1000):
	if not isinstance(inPs, list):
		inPs = [inPs]
	for inP in inPs: 
		if isConnected(outP, inP) == True:
			print outP.get_port_profile().name,'and',inP.get_port_profile().name,'are already connected'
			continue
		if dataTypeOfPort(outP) != dataTypeOfPort(inP):
			print outP.get_port_profile().name,'and',inP.get_port_profile().name,'have different data types'
			continue
		nv1 = SDOPackage.NameValue("dataport.interface_type", 
					   any.to_any("corba_cdr"))
		nv2 = SDOPackage.NameValue("dataport.dataflow_type", 
					   any.to_any(dataflow))
		nv3 = SDOPackage.NameValue("dataport.subscription_type", 
					   any.to_any(subscription))
		nv4 = SDOPackage.NameValue("dataport.buffer.length", 
					   any.to_any(str(bufferlength)))
		nv5 = SDOPackage.NameValue("dataport.publisher.push_rate", 
					   any.to_any(str(rate)))
		con_prof = RTC.ConnectorProfile("connector0", "", [outP, inP], 
						[nv1, nv2, nv3, nv4, nv5])
		ret,prof = inP.connect(con_prof)
		if ret != RTC.RTC_OK:
			print "failed to connect"
			continue
		# confirm connection
		if isConnected(outP, inP) == False:
			print "connet() returned RTC_OK, but not connected"

##
# \brief convert data into CDR format
# \param data data to be converted
# \return converted data in CDR format
#
def data2cdr(data):
	return cdrMarshal(any.to_any(data).typecode(), data, True)

##
# \brief convert data from CDR format
# \param cdr in CDR format 
# \param classname class name of the data
# \return converted data
#
def cdr2data(cdr, classname):
	return cdrUnmarshal(any.to_any(eval(classname)).typecode(), cdr, True)

##
# \brief write data to a data port	
# \param port reference of data port
# \param data data to be written
# \param tm after this time, a connection to write data is disconnected
#
def writeDataPort(port, data, tm=1.0):
	nv1 = SDOPackage.NameValue("dataport.interface_type", 
				   any.to_any("corba_cdr"))
	nv2 = SDOPackage.NameValue("dataport.dataflow_type", 
				   any.to_any("Push"))
	nv3 = SDOPackage.NameValue("dataport.subscription_type", 
				   any.to_any("flush"))
	con_prof = RTC.ConnectorProfile("connector0", "", [port], 
					[nv1, nv2, nv3])
	ret,prof = port.connect(con_prof)
	if ret != RTC.RTC_OK:
		print "failed to connect"
		return None
	for p in prof.properties:
		if p.name == 'dataport.corba_cdr.inport_ior':
			ior = any.from_any(p.value)
			obj = orb.string_to_object(ior)
			inport = obj._narrow(InPortCdr)
			cdr = data2cdr(data)
			if inport.put(cdr) != OpenRTM.PORT_OK:
				print "failed to put"
			time.sleep(tm)
			port.disconnect(prof.connector_id)
	return None		
			
		
##
# \brief read data from a data port	
# \param port reference of data port
# \param timeout timeout[s] 
# \return data 
def readDataPort(port, timeout = 1.0):
	pprof = port.get_port_profile()
	for prop in pprof.properties:
		if prop.name == "dataport.data_type":
			classname = any.from_any(prop.value)
			break;
	nv1 = SDOPackage.NameValue("dataport.interface_type", 
				   any.to_any("corba_cdr"))
	nv2 = SDOPackage.NameValue("dataport.dataflow_type", 
				   any.to_any("Pull"))
	nv3 = SDOPackage.NameValue("dataport.subscription_type", 
				   any.to_any("flush"))
	con_prof = RTC.ConnectorProfile("connector0", "", [port], 
					[nv1, nv2, nv3])
	ret,prof = port.connect(con_prof)
	#
	if ret != RTC.RTC_OK:
		print "failed to connect"
		return None
	for p in prof.properties:
		#print p.name
		if p.name == 'dataport.corba_cdr.outport_ior':
			ior = any.from_any(p.value)
			obj = orb.string_to_object(ior)
			outport = obj._narrow(OutPortCdr)
			tm = 0
			while tm < timeout:
				try:
					ret,data = outport.get()
					if ret == OpenRTM.PORT_OK:
						port.disconnect(prof.connector_id)
						print classname
						print eval(classname)
						return cdr2data(data, classname)
				except:
					pass
				time.sleep(0.1)
				tm = tm + 0.1

	port.disconnect(prof.connector_id)
	return None		
			
		

##
# \brief get a service of RT component
# \param rtc IOR of RT component
# \param svcname name of the service
# \return IOR of the service
#
def findService(rtc, svcname):
	prof = rtc.get_component_profile()
	#print "RTC name:",prof.instance_name
	port_prof = prof.port_profiles
	port = None
	for pp in port_prof:
		#print "name:",pp.name
		ifs = pp.interfaces
		for aif in ifs:
			#print "IF name:",aif.instance_name
			#print "IF type:",aif.type_name
			if aif.instance_name == svcname:
				port = pp.port_ref
	con_prof = RTC.ConnectorProfile("noname","",[port],[])
	ret, con_prof = port.connect(con_prof)
	ior = any.from_any(con_prof.properties[0].value)
	return orb.string_to_object(ior)

##
# \brief update default configuration set
# \param rtc IOR of RT component
# \param nvlist list of pairs of name and value
#
def setConfiguration(rtc, nvlist):
	cfg = rtc.get_configuration()
	cfgsets = cfg.get_configuration_sets()
	if len(cfgsets) == 0:
		print "configuration set is not found"
		return
	cfgset = cfgsets[0]
	for nv in nvlist:
		name = nv[0]
		value = nv[1]
		for d in cfgset.configuration_data:
			if d.name == name:
				d.value = any.to_any(value)
				cfg.set_configuration_set_values(cfgset)
				break;
	cfg.activate_configuration_set('default')

##
# \brief narrow ior
# \param ior ior
# \param klass class name 
# \param package package where the class is defined
def narrow(ior, klass, package="OpenHRP"):
	return ior._narrow(getattr(sys.modules[package], klass))

initCORBA()
