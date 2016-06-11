/*
 * (C) Copyright 2014 Aurélien Rodot. All rights reserved.
 *
 * This file is part of the Gamebuino Library (http://gamebuino.com)
 *
 * The Gamebuino Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include "Sound.h"


#define SOUND_COMMANDS 0

#define SOUNDPOINTER (231 * 128)


uint8_t globalVolume = 1;
uint8_t _rand = 0;
uint8_t _chanCount[NUM_CHANNELS]; //counts until the next change of the waveform (half period)
boolean _chanState[NUM_CHANNELS]; //if the waveform is currently high or low
uint8_t _chanHalfPeriod[NUM_CHANNELS]; //duration of half the period of the waveform
uint8_t _chanOutputVolume[NUM_CHANNELS]; //amplitude of the outputted waveform

uint8_t _chanOutput[NUM_CHANNELS]; //current value of the outputted waveform (what actually gets send to the speaker)

#if SOUND_COMMANDS
	// commands data
	int8_t commandsCounter[NUM_CHANNELS];
	int8_t volumeSlideStepDuration[NUM_CHANNELS];
	int8_t volumeSlideStepSize[NUM_CHANNELS];
	uint8_t arpeggioStepDuration[NUM_CHANNELS];
	int8_t arpeggioStepSize[NUM_CHANNELS];
	uint8_t tremoloStepDuration[NUM_CHANNELS];
	int8_t tremoloStepSize[NUM_CHANNELS];
#endif

//tracks data
uint16_t *trackData[NUM_CHANNELS];

// pattern data
uint16_t *patternData[NUM_CHANNELS];

// note data
int8_t noteVolume[NUM_CHANNELS];
uint8_t notePitch[NUM_CHANNELS];
uint8_t noteDuration[NUM_CHANNELS]; // how long a note still needs to be played
#define WAVEFORM_SQUARE 0
#define WAVEFORM_NOISE 1
uint8_t _chanWaveform[NUM_CHANNELS];


#if(EXTENDED_NOTE_RANGE > 0)
//extended note range
#define NUM_PITCH 59
const uint8_t _halfPeriods[NUM_PITCH] PROGMEM= {246,232,219,207,195,184,174,164,155,146,138,130,123,116,110,104,98,92,87,82,78,73,69,65,62,58,55,52,49,46,44,41,39,37,35,33,31,29,28,26,25,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6};
#else
//regular note range
#define NUM_PITCH 36
const uint8_t _halfPeriods[NUM_PITCH] PROGMEM= {246,232,219,207,195,184,174,164,155,146,138,130,123,116,110,104,98,92,87,82,78,73,69,65,62,58,55,52,49,46,44,41,39,37,35,33};
#endif

void Sound::begin(){
#if(NUM_CHANNELS > 0)
	for(byte i=0; i<NUM_CHANNELS; i++){
		patternData[i] = trackData[i] = 0;
		_chanOutputVolume[i] =  0;
		//changeInstrumentSet(defaultInstruments, i); //load default instruments. #0:square wave, #1: noise
		//command(CMD_INSTRUMENT, 0, 0, i); //set the default instrument to square wave
	}
	
	analogWrite(3, 1); //lazy version to get the right register settings for PWM (hem)
	TCCR2B = (TCCR2B & B11111000) | 1; //set timer 2 prescaler to 1 -> 30kHz PWM on pin 3


	// initialize timer1 
	noInterrupts(); // disable all interrupts
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0;

	OCR1A = 280; // compare match register
	TCCR1B |= (1 << WGM12); // CTC mode
	TCCR1B |= (1 << CS10); // 1 prescaler
	TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
	interrupts(); // enable all interrupts
#endif
}

void Sound::command(uint8_t cmd, uint8_t X, int8_t Y, uint8_t i){
#if(NUM_CHANNELS > 0)
	if(i>=NUM_CHANNELS)
		return;
	switch(cmd){
	case CMD_VOLUME: //volume
		X = constrain(X, 0, 10);
		noteVolume[i] = X;
		break;
	case CMD_INSTRUMENT: //instrument
		_chanWaveform[i] = X;
		break;
	#if SOUND_COMMANDS
	case CMD_SLIDE: //volume slide
		volumeSlideStepDuration[i] = X;
		volumeSlideStepSize[i] = Y;
		break;
	case CMD_ARPEGGIO:
		arpeggioStepDuration[i] = X;
		arpeggioStepSize[i] = Y;
		break;
	case CMD_TREMOLO:
		tremoloStepDuration[i] = X;
		tremoloStepSize[i] = Y;
		break;
	#endif
	}
#endif
}

void Sound::playTrack(uint8_t channel){
	Sound::stopTrack(channel);
	trackData[channel] = (uint16_t*)(SOUNDPOINTER + (channel*42));
}

void Sound::updateTrack(uint8_t channel){
	if(trackData[channel] && !patternData[channel]){
		uint16_t data = pgm_read_word(trackData[channel]);
		trackData[channel]++;
		if(data == 0xFFFF){ //end of the track
			trackData[channel] = 0;
			//Serial.println("track end");
			Sound::stopNote(channel);
			Sound::playTrack(channel);
			return;
		}
		uint8_t patternID = data & 0xFF;
		//Serial.print(channel);
		//Serial.print(" pattern # ");
		//Serial.println(patternID, HEX);
		//data >>= 8;
		//patternPitch[channel] = data;
		//&((const uint16_t* const*)((231 * 128)+80)[patternID])
//		Serial.println((uint16_t)(((const uint16_t* const*)((231 * 128)+80)) + patternID));
//		Serial.println(pgm_read_word((((const uint16_t* const*)((231 * 128)+80)) + patternID)));
		playPattern((const uint16_t*)pgm_read_word((((const uint16_t* const*)(SOUNDPOINTER+42+42)) + patternID)), channel);
	}
}
void Sound::updateTrack(){
#if(NUM_CHANNELS > 0)
	for (uint8_t i=0; i<NUM_CHANNELS; i++){
		Sound::updateTrack(i);
	}
#endif
}
void Sound::stopTrack(uint8_t channel){
	trackData[channel] = 0;
	stopPattern(channel);
}

void Sound::playPattern(const uint16_t* pattern, uint8_t channel){
#if(NUM_CHANNELS > 0)
	Sound::stopPattern(channel);
	patternData[channel] = (uint16_t*)pattern;
	noteVolume[channel] = 9;
	//reinit commands
#if SOUND_COMMANDS
	volumeSlideStepDuration[channel] = 0;
	arpeggioStepDuration[channel] = 0;
	tremoloStepDuration[channel] = 0;
#endif
	//Serial.print("play pattern\n");
#endif
}
void Sound::stopPattern(uint8_t channel){
#if(NUM_CHANNELS > 0)
	Sound::stopNote(channel);
	patternData[channel] = 0;
#endif
}
void Sound::updatePattern(uint8_t i){
#if(NUM_CHANNELS > 0)
	if(patternData[i]){
		if(noteDuration[i]==0){//if the end of the previous note is reached
			
			uint16_t data = pgm_read_word(patternData[i]);
			patternData[i]++;
			
			if(data == 0){ //end of the pattern reached
				patternData[i] = 0;
				if(trackData[i]){ //if this pattern is part of a track, get the next pattern
					updateTrack(i);
					if(!patternData[i]){
						return;
					}
					data = pgm_read_word(patternData[i]);
				} else {
					Sound::stopNote(i);
					//Serial.print("pattern end\n");
					return;
				}
			}

			while (data & 0x0001){ //read all commands and instrument changes
				data >>= 2;
				//Serial.print("\ncmd\t");
				uint8_t cmd = data & 0x0F;
				data >>= 4;
				uint8_t X = data & 0x1F;
				data >>= 5;
				int8_t Y = data - 16;
				command(cmd,X,Y,i);
				//Serial.print(cmd);
				//Serial.print("\t");
				//Serial.print(data & 0x1F);
				//Serial.print("\t");
				//Serial.println(int8_t(data >> 5) - 16);
				data = pgm_read_word(patternData[i]);
				patternData[i]++;
			}
			data >>= 2;

			uint8_t pitch = data & 0x003F;
			data >>= 6;
			
			uint8_t duration = data;
			if(pitch != 63){
			//Serial.print(i);
			//Serial.print("\tnote");
			//Serial.print("\t");
			//Serial.print(duration);
			//Serial.print("\t");
			//Serial.print(volume);
			//Serial.print("\t");
			//Serial.print(pitch);
			//Serial.print("\t");
			//Serial.print(instrumentID);
			//Serial.print("\n");
			}
			
			playNote(pitch, duration, i);
		}
	}
#endif
}
void Sound::updatePattern(){
#if(NUM_CHANNELS > 0)
	for (uint8_t i=0; i<NUM_CHANNELS; i++){
		Sound::updatePattern(i);
	}
#endif
}


void Sound::playNote(uint8_t pitch, uint8_t duration, uint8_t channel){
#if(NUM_CHANNELS > 0)
	//set note
	notePitch[channel] = pitch;
	noteDuration[channel] = duration;
	//reinit vars
#if SOUND_COMMANDS
	commandsCounter[channel] = 0;
#endif
#endif
}


void Sound::updateNote(uint8_t i){
#if(NUM_CHANNELS > 0)
	if(noteDuration[i]){
		//Serial.println("updating note");
		//Serial.println(noteDuration[i]);
		if(!--noteDuration[i]){
			Sound::stopNote(i);
			return;
		}
	#if SOUND_COMMANDS
		commandsCounter[i]++;
		_chanHalfPeriod[i] = notePitch[i];
		if(arpeggioStepDuration[i]){
			_chanHalfPeriod[i] += commandsCounter[i] / arpeggioStepDuration[i] * arpeggioStepSize[i];
		}
		_chanHalfPeriod[i] &=  NUM_PITCH; //wrap

		_chanOutput[i] = noteVolume[i] * 7;
		if(volumeSlideStepDuration[i]){
			_chanOutput[i] += commandsCounter[i] / volumeSlideStepDuration[i] * volumeSlideStepSize[i];
		}
		if(tremoloStepDuration[i]){
			_chanOutput[i] += ((commandsCounter[i]/tremoloStepDuration[i]) % 2) * tremoloStepSize[i];
		}
		_chanOutput[i] = constrain(_chanOutput[i], 0, 9);
		if(_chanHalfPeriod[i] == 63){
			_chanOutput[i] = 0;
		}


		_chanHalfPeriod[i] = pgm_read_byte(_halfPeriods + _chanHalfPeriod[i]);
		_chanOutput[i] = _chanOutputVolume[i] = _chanOutput[i] * globalVolume * VOLUME_CHANNEL_MAX;
	#else
		_chanHalfPeriod[i] = notePitch[i];
		_chanHalfPeriod[i] %= NUM_PITCH; //wrap
		
		_chanOutput[i] = noteVolume[i] * 7;
		if(_chanHalfPeriod[i] == 63){
			_chanOutput[i] = 0;
		}

		_chanHalfPeriod[i] = pgm_read_byte(_halfPeriods + _chanHalfPeriod[i]);
		_chanOutput[i] = _chanOutputVolume[i] = _chanOutput[i] * globalVolume * VOLUME_CHANNEL_MAX;
	#endif
	}

#endif
}
void Sound::updateNote(){
#if(NUM_CHANNELS > 0)
	for (uint8_t i=0; i<NUM_CHANNELS; i++){
		Sound::updateNote(i);
	}
#endif
}
void Sound::stopNote(uint8_t channel) {
#if(NUM_CHANNELS > 0)
	if(channel>=NUM_CHANNELS)
		return;
	noteDuration[channel] = 0;
#if SOUND_COMMANDS
	commandsCounter[channel] = 0;
#endif
	//output
	_chanOutput[channel] = 0;
	_chanOutputVolume[channel] = 0;
	//Sound::generateOutput();
#endif
}


uint16_t counter = 3000;
ISR(TIMER1_COMPA_vect){ // timer compare interrupt service routine
#if(NUM_CHANNELS > 0)
	Sound::generateOutput();
	if(!counter--){
		counter = 3000;
		Sound::updateTrack();
		Sound::updatePattern();
		Sound::updateNote();
	}
#endif
}

void __attribute__((optimize("O3"))) Sound::generateOutput() {
#if(NUM_CHANNELS > 0)
	boolean outputChanged = false;
	//no for loop here, for the performance sake (this function runs 15 000 times per second...)
	//CHANNEL 0
	if (_chanOutputVolume[0]) {
		_chanCount[0]++;
		if (_chanCount[0] >= _chanHalfPeriod[0]) {
			//Serial.println("====");
			outputChanged = true;
			_chanState[0] = !_chanState[0];
			_chanCount[0] = 0;
			if (_chanWaveform[0] == WAVEFORM_NOISE) {
				_rand = 67 * _rand + 71;
				_chanOutput[0] = _rand % _chanOutputVolume[0];
			}
		}
	}


	//CHANNEL 1
	#if (NUM_CHANNELS > 1)
	if (_chanOutputVolume[1]) {
		_chanCount[1]++;
		if (_chanCount[1] >= _chanHalfPeriod[1]) {
			outputChanged = true;
			_chanState[1] = !_chanState[1];
			_chanCount[1] = 0;
			if (_chanWaveform[1] == WAVEFORM_NOISE) {
				_rand = 67 * _rand + 71;
				_chanOutput[1] = _rand % _chanOutputVolume[1];
			}
		}
	}
	#endif

	//CHANNEL 2
	#if (NUM_CHANNELS > 2)
	if (_chanOutputVolume[2]) {
		_chanCount[2]++;
		if (_chanCount[2] >= _chanHalfPeriod[2]) {
			outputChanged = true;
			_chanState[2] = !_chanState[2];
			_chanCount[2] = 0;
			if (_chanWaveform[2] == WAVEFORM_NOISE) {
				_rand = 67 * _rand + 71;
				_chanOutput[2] = _rand % _chanOutputVolume[2];
			}
		}
	}
	#endif

	//CHANNEL 3
	#if (NUM_CHANNELS > 3)
	if (_chanOutputVolume[3]) {
		_chanCount[3]++;
		if (_chanCount[3] >= _chanHalfPeriod[3]) {
			outputChanged = true;
			_chanState[3] = !_chanState[3];
			_chanCount[3] = 0;
			if (_chanWaveform[3] == WAVEFORM_NOISE) {
				_rand = 67 * _rand + 71;
				_chanOutput[3] = _rand % _chanOutputVolume[3];
			}
		}
	}
	#endif

	if (outputChanged) {
		uint8_t output = 0;

		//CHANNEL 0
		if(_chanState[0]){
			output += _chanOutput[0];
		}

		//CHANNEL 1
		#if (NUM_CHANNELS > 1)
		if(_chanState[1]){
			output += _chanOutput[1];
		}
		#endif
		
		//CHANNEL 2
		#if (NUM_CHANNELS > 2)
		if(_chanState[2]){
			output += _chanOutput[2];
		}
		#endif
		
		//CHANNEL 3
		#if (NUM_CHANNELS > 3)
		if(_chanState[3]){
			output += _chanOutput[3];
		}
		#endif

		OCR2B = output; //60x faster than analogOutput() !
	}
#endif
}
