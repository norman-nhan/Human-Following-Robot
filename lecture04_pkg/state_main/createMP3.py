#!/usr/bin/env python3 
from gtts import gTTS

def main():
    speech = 'AHHHHH!!!! PLEASE COMEBACK'
    tts = gTTS(f'{speech}', lang="en")
    file_name = "scream"
    tts.save(f'{file_name}.mp3')

if __name__ == "__main__":
    main()
