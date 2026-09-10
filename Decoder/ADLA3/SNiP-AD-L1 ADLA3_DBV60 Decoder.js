/* Decoder for SNiP-AD-L1 AD-NODE-L1 with DBV60 Dendrometer Sensor.
	Suitable for AD-NODE Model revision ADLA3.
	DBV60 Sensor powered via the 12VDC pin, and analog output of sensor connected to input channel "V0".
	NOTE: Decoder Not suitable for ADLA1 and ADLA2 model.
*/

//Change log
/*
20260828: 
Toby Partridge ICT International
	* Re-labelled inputs description to better match to the physical label on the circuit board silk screen.
	* Fixed Digital input name and conversion as there was an error converting from 0,1,2,3 into the correct parsed order.
	* Added DBV60 Code segment to decoder section with reference to correct analog input channel "V0".

2026/09/10:
ICT International Toby Partridge
	* Increased toFixed from toFixed (3) to toFixed (6), before the “3” was sacrificing accuracy by pushing the result to three digits after the decimal place. 
	  Six digits after the decimal place allows maintaining of the accuracy during the calculation.
*/

// Structure Type Define, 'nested' or 'flat'
var TYPE = 'nested';

/* Class Buffer
	Purpose: A psuedo buffer class for accessing packet data,
		allows for uniformity between decoder types
*/
function Buf(buf){this.pl=buf; this.length=this.pl.length;}
Buf.prototype.readUInt8=function(ofs){return ((this.pl[ofs]<<24)>>>24);};
Buf.prototype.readUInt16BE=function(ofs){return ((this.pl[ofs++]<<24|this.pl[ofs++]<<16)>>>16);};
Buf.prototype.readUInt32BE=function(ofs){return ((this.pl[ofs++]<<24|this.pl[ofs++]<<16|this.pl[ofs++]<<8|this.pl[ofs++])>>>0);};
Buf.prototype.readUInt16LE=function(ofs){return ((this.pl[ofs+1]<<24|this.pl[ofs--]<<16)>>>16);};
Buf.prototype.readUInt32LE=function(ofs){return ((this.pl[ofs+3]<<24|this.pl[ofs+2]<<16|this.pl[ofs+1]<<8|this.pl[ofs])>>>0);};
Buf.prototype.readInt8=function(ofs){return ((this.pl[ofs]<<24)>>24);};
Buf.prototype.readInt16BE=function(ofs){return ((this.pl[ofs++]<<24|this.pl[ofs++]<<16)>>16);};
Buf.prototype.readInt32BE=function(ofs){return ((this.pl[ofs++]<<24|this.pl[ofs++]<<16|this.pl[ofs++]<<8|this.pl[ofs++])>>0);};
Buf.prototype.readInt16LE=function(ofs){return ((this.pl[ofs+1]<<24|this.pl[ofs--]<<16)>>16);};
Buf.prototype.readInt32LE=function(ofs){return ((this.pl[ofs+3]<<24|this.pl[ofs+2]<<16|this.pl[ofs+1]<<8|this.pl[ofs])>>0);};
Buf.prototype.readFloatBE=function(ofs){return B2Fl(this.readUInt32BE(ofs));};
Buf.prototype.readFloatLE=function(ofs){return B2Fl(this.readUInt32LE(ofs));};
Buf.prototype.slice=function(s,e){return this.pl.slice(s,e);};
Buf.prototype.length=function(){return this.pl.length;};

/*Function Bytes2Float32(bytes)
	Purpose: Decodes an array of bytes(len 4(32 bit) into a float.
	Args:	bytes - an array of bytes, 4 bytes long
	Returns: 32bit Float representation
*/
function B2Fl(b){
	var sign =(b>>31)?-1:1;
	var exp=((b>>23)&0xFF)-127;
	var sig=(b&~(-1<<23));
	if(exp==128) return sign*((sig)?Number.NaN:Number.POSITIVE_INFINITY);
  if(exp==-127){
		if(sig===0) return sign*0.0;
		exp=-126;
		sig/=(1<<22);
	} else sig=(sig|(1<<23))/(1<<23);
	return sign*sig*Math.pow(2,exp);
}

/*Function buildNested(a)
	Purpose: Takes an array and parses them into a clean and succinct object of nested parameter sets
	Args: a - An array of arrays containing Parameter Sets
	Returns: An Object containing nested Parameter Sets
*/
function buildNested(a){
	var exc=["main","diagnostic","downlink","device_info","unknown"];
	var ret=[];
	for(var el in a){
		var e=a[el];
		var par={};
		par['label']=e[0];
		par['channelId']=e[1];
		par['value']=e[2];
		var e_length = e.length;
		if(e_length>3&&exc.indexOf(e[3])<0) par['source']=e[3];
		if(e_length>4) par['unit']=e[4];
		if(e_length>5) par['address']=e[5];
		ret.push(par);
	} return ret;}

/*Function buildFlat(a)
	Purpose: Takes an array and parses them into a clean and succinct object of flat parameters
	Args: a - An array of arrays containing Parameter Sets
	Returns: An Object containing nested Parameter Sets
*/
function buildFlat(a){
	var exc=["main","diagnostic","downlink","device_info","unknown"];
	var ret={};
	for(var el in a){
		var e=a[el];
		var label = '';
		if(e.length==6){
			e[0] = e[0]+e[5]+'_';
		}
		if(e.length<=4){
			label=(exc.indexOf(e[3])<0) ? (e[0]+e[1]) : (e[0]);
		} else{
			label=(exc.indexOf(e[3])<0) ? (e[0]+e[1]+'_'+e[4]) : (e[0]+'_'+e[4]);

		} ret[label]=e[2];
	} return ret;}

//Function - Decode, Wraps the primary decoder function for Chirpstack
function Decode(fPort, bytes){
	var buf = new Buf(bytes);
	var decoded = {};
	var readingsArr = primaryDecoder(buf, fPort);
	if(TYPE == 'flat'){ decoded = buildFlat(readingsArr); }
	else decoded['data'] = buildNested(readingsArr);
	
	return decoded;
}

//Function - parseDeviceMsg, Wraps the primary decoder function for NNNCo
function parseDeviceMsg(buf, lMsg){
	var p = lMsg.loraPort;
	var readingArr = primaryDecoder(buf, p);
	var readingList = buildNested(readingArr);
	return readingList;
}

//Function - Decoder, Wraps the primary decoder function for TTNv3
function Decoder(b, p){
	var buf = new Buf(b);
	var decoded = {};
	var readingsArr = primaryDecoder(buf, p);
	if(TYPE == 'flat'){ decoded = buildFlat(readingsArr); }
	else decoded['data'] = buildNested(readingsArr);
	
	return decoded;
}

/*Function primaryDecoder
	Purpose: Main Entry point of TTN Console Decoder
	Args:	bytes - An array of bytes from LoRaWan raw payload(Hex Represented)
			port - LoRaWan Port that the message came through(set by Definium firmware)
	Returns: decoded - An object with data fields as decoded parameter values
*/
function primaryDecoder(buf,p){
	var arr = [];
	var byte = 0;

	// Data Packet Response Received
	if(p === 1){
		var src = "main";
		arr.push(['packet-type', 0, "DATA_PACKET", src]);
		src = "diagnostic";
		
		arr.push(["uptime", 0, +(buf.readUInt32LE(byte)), src, "s"]);
		arr.push(["battery-voltage", 0, +((buf.readUInt16LE(byte = byte+4)/1000).toFixed(3)), src, "V"]);
		
		src = "adc";
		
		// Create var objects to store data from bytes
		var current0 = +(buf.readUInt16LE(byte = byte+2)); // var to store current sensor input "I"
		var voltage0 = +(buf.readUInt32LE(byte = byte+2)); // var to store voltage sensor input "V0"
		var voltage1 = +(buf.readUInt32LE(byte = byte+4)); // var to store voltage sensor input "V1"
		// Convert and push values to array.push
		arr.push(["current-adc", 0, current0, src, "uA"]); // array pushed uA from 4-20mA input "I"
		arr.push(["voltage-adc", 0, voltage0, src, "uV"]); // array pushed uV microVolt input "V0"
		arr.push(["voltage-adc", 1, voltage1, src, "uV"]); // array pushed uV microVolt input "V1"
		arr.push(["temperature", 1, +((buf.readInt32LE(byte = byte+4)/1000).toFixed(3)), src, "C"]); // thermistor temperature "T1" 
		//Note: T0 input is skipped on ADLA3 models, the channel is repurposed for ADC voltage input "V1" and firmware locked.
		
		// DBV60 Sensor Equation
		var v_in = voltage0;
		// Convert uV to mV
		v_in = v_in / 1000;
		
		var stem_circumference = +((v_in * 0.0254).toFixed(6));
		arr.push(["stem-circumference", 0, stem_circumference, src, "mm"]);
		
		src = "digital";
		var digital0 = +(buf.readUInt32LE(byte=byte+4)); // var to store digital pulse counter input "D0"
		var digital1 = +(buf.readUInt32LE(byte=byte+4)); // var to store digital pulse counter input "D1"
		var digital2 = +(buf.readUInt32LE(byte=byte+4)); // var to store digital pulse counter input "D2"
		var digital3 = +(buf.readUInt32LE(byte=byte+4)); // var to store digital pulse counter input "D3"
		// Convert and push values to array.push
		arr.push(["digital-count", 0, digital0, src]);	 // array pushed digital pulse counter input "D0"
		arr.push(["digital-count", 1, digital1, src]);   // array pushed digital pulse counter input "D1"
		//arr.push(["digital-count", 2, digital2, src]); // Uncomment this line if pulse counter "D2" is required.
		//arr.push(["digital-count", 3, digital3, src]); // Uncomment this line if pulse counter "D3" is required.
		
		// Accelerometer values
		var mult = (2.0 / 32678.0) * 1000;
		src = "main";	
	    var x = +((buf.readInt16LE(byte = byte+4) * mult).toFixed(3));
		var y = +((buf.readInt16LE(byte = byte+2) * mult).toFixed(3));
		var z = +((buf.readInt16LE(byte = byte+2) * mult).toFixed(3));
		//arr.push(["accelerometer-x", 0, x, src, "mG"]); Commented out to reduce MQTT payload unnecessary values.  
		//arr.push(["accelerometer-y", 0, y, src, "mG"]); Commented out to reduce MQTT payload unnecessary values.  
		//arr.push(["accelerometer-z", 0, z, src, "mG"]); Commented out to reduce MQTT payload unnecessary values.  
		var roll = Math.abs(Math.atan((x)/(y))*(180/Math.PI));
		var pitch = Math.abs(Math.atan((x)/(z))*(180/Math.PI));
		var x_offset = +((y < 0) ? (0 + (90-roll)).toFixed(3) : (0 - (90-roll)).toFixed(3));
		var y_offset = +((z < 0) ? (0 + (90-pitch)).toFixed(3) : (0 - (90-pitch)).toFixed(3));
		arr.push(["x-tilt-offset", 0, x_offset, src, "Degrees"]);
		arr.push(["y-tilt-offset", 0, y_offset, src, "Degrees"]);
	} else if(p === 10){
		//Device Info Packet Recieved
		var src = "device_info";
		arr.push(["packet-type", 0, "DEVICE_INFO", "main"]);
		var productid = (["product-id", 0, buf.readUInt32BE(byte), src]);
		var batchnumber = (["batch-number", 0, buf.readUInt32BE(byte=byte+4), src]);
		arr.push(["software-version", 0, buf.readUInt32BE(byte=byte+4), src]);
	} else if(p === 100){
		//Downlink Response Packet Recieved
		arr.push(["packet-type", 0, "DOWNLINK_RESPONSE", "main"]);
		arr.push(["downlink-response", 0, String.fromCharCode.apply(String, buf.slice(0, buf.length)), "downlink"]);
	} else{
		// Unknown Response Recieved
		arr.push(["packet-type", 0, "UNKNOWN_RESPONSE", "main"]);
		//arr.push(["raw-payload", 0, buf.slice(0, buf.length), "unknown"]);
	}
	return arr;
}
