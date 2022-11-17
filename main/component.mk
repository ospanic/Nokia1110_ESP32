#
# Main component makefile.
#
# This Makefile can be left empty. By default, it will take the sources in the
# src/ directory, compile them and link them into lib(subdirectory_name).a
# in the build directory. This behaviour is entirely configurable,
# please read the ESP-IDF documents if you need to do this.
#
COMPONENT_EMBED_TXTFILES := nokia.pcm  00.bin 01.bin 02.bin 03.bin 04.bin 05.bin 06.bin 07.bin 08.bin 
COMPONENT_EMBED_TXTFILES += 09.bin 10.bin 11.bin 12.bin 13.bin 14.bin 15.bin 16.bin 
COMPONENT_EMBED_TXTFILES += 17.bin 18.bin 19.bin 20.bin 21.bin 22.bin desktop.bin msg.bin game.bin
COMPONENT_EMBED_TXTFILES += game1.bin game2.bin game3.bin game4.bin game5.bin game6.bin
COMPONENT_EMBED_TXTFILES += game1s.bin game2s.bin game3s.bin game4s.bin game5s.bin game6s.bin