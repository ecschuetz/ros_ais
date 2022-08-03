#AIS Standard Class B Equipment Position Report (Message 18)
 
Header header

int32 id   #Identifier for this message 1, 2 or 3

int32 repeat_indicator   #Used by the repeater to indicate how many times a message has been repeated. See Section 4.6.1, Annex 2; 0-3; 0 = default; 3 = do not repeat any more.

int32 mmsi   #MMSI number





int32 length 
int32 beam
int32 draught


