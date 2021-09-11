function Decoder(bytes, port) {
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  var decoded = {};
  
  function unpack32(offset) { return ((bytes[offset]<<24)>>>0)+((bytes[offset+1]<<16)>>>0)+((bytes[offset+2]<<8)>>>0)+(bytes[offset+3]>>>0); }
  function unpack16(offset) { return ((bytes[offset]<<8)>>>0)+(bytes[offset+1]>>>0); }
  function unpack16s(offset) {
      if (bytes[offset] & 0x80) {
	  // negative, need to use twos complement invert
	  var bits = unpack16(offset);
	  return 0 - ((bits ^ 0xFFFF)+1);
      } else {
	  // positive, same as unsigned
	  return unpack16(offset);
      }
  }

  if (port === 1) {
      if (bytes[0] == 1) {
	  // Data format v1
	  decoded.vers = bytes[0];
	  decoded.batt = bytes[1];
	  decoded.hum = unpack16(2)/10.0;
	  decoded.temp = unpack16s(4)/10.0;
	  decoded.raincount = unpack32(6);
	  decoded.hailcount = unpack32(10);
      }
      else if (bytes.length==12) {
	  // unversioned data, early prototypes
	  decoded.vers = 0;
	  decoded.hum = unpack16(0)/10.0;
	  decoded.temp = unpack16s(2)/10.0;
	  decoded.raincount = unpack32(4);
	  decoded.hailcount = unpack32(8);
      }
  }

  return decoded;
}
