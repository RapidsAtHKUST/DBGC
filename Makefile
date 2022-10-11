CC = g++
FLAGS = -std=c++17 -O3 -g -Wall -I. -DENABLE_TBB
LIBS = -lstdc++fs -lbz2 -llzma -lz -ltbb -pthread \
		-Lbuild -Wl,-rpath=${CURDIR}/build -lDBGC

BUILD = build
OBJ = build/obj
BUILD_TESTS = build/tests

COMMON = common
COMPRESS = compressors
PARTITION = partition
TESTS = tests

all : dir $(BUILD)/client \
		$(BUILD)/server \
		$(BUILD)/lidarpcc_compress \
		$(BUILD)/lidarpcc_decompress

to_db : dir $(BUILD)/client \
		$(BUILD)/server_db \
		$(BUILD)/lidarpcc_compress \
		$(BUILD)/lidarpcc_decompress

dir: $(OBJ) $(BUILD_TESTS)

$(OBJ) :
	mkdir -p $(OBJ)

$(BUILD_TESTS) :
	mkdir -p $(BUILD_TESTS)

#################### executable ####################

$(BUILD)/client : $(OBJ)/client.o \
		$(BUILD)/libDBGC.so
	$(CC) $(FLAGS) $(OBJ)/client.o -o $@ $(LIBS)

$(BUILD)/server_db : $(OBJ)/server_db.o \
		$(BUILD)/libDBGC.so
	$(CC) $(FLAGS) $(OBJ)/server_db.o -o $@ $(LIBS) -lodbc

$(BUILD)/server : $(OBJ)/server.o \
		$(BUILD)/libDBGC.so
	$(CC) $(FLAGS) $(OBJ)/server.o -o $@ $(LIBS)

$(BUILD)/lidarpcc_compress : $(OBJ)/lidarpcc_compress.o \
		$(BUILD)/libDBGC.so
	$(CC) $(FLAGS) $(OBJ)/lidarpcc_compress.o -o $@ $(LIBS)

$(BUILD)/lidarpcc_decompress : $(OBJ)/lidarpcc_decompress.o \
		$(BUILD)/libDBGC.so
	$(CC) $(FLAGS) $(OBJ)/lidarpcc_decompress.o -o $@ $(LIBS)

#################### test object ####################

$(OBJ)/client.o : $(TESTS)/client.cpp \
		$(COMMON)/CLI11.hpp $(COMMON)/utils.h \
		$(COMMON)/types.h $(COMMON)/file_io.h \
		$(COMPRESS)/base.h \
		$(COMPRESS)/general.h \
		$(COMPRESS)/adaptive_compressor.h
	$(CC) -c $(FLAGS) $(TESTS)/client.cpp -fPIC -o $@

$(OBJ)/server.o : $(TESTS)/server.cpp \
		$(COMMON)/CLI11.hpp $(COMMON)/utils.h \
		$(COMMON)/file_io.h $(COMMON)/types.h \
		$(COMPRESS)/adaptive_compressor.h
	$(CC) -c $(FLAGS) $(TESTS)/server.cpp -fPIC -o $@

$(OBJ)/server_db.o : $(TESTS)/server.cpp \
		$(COMMON)/CLI11.hpp $(COMMON)/utils.h \
		$(COMMON)/file_io.h $(COMMON)/types.h \
		$(COMPRESS)/adaptive_compressor.h
	$(CC) -c $(FLAGS) $(TESTS)/server.cpp -DTO_DB -fPIC -o $@

$(OBJ)/lidarpcc_compress.o : $(TESTS)/lidarpcc_compress.cpp \
		$(COMMON)/CLI11.hpp $(COMMON)/utils.h \
		$(COMMON)/types.h $(COMMON)/file_io.h \
		$(COMPRESS)/base.h \
		$(COMPRESS)/general.h \
		$(COMPRESS)/adaptive_compressor.h
	$(CC) -c $(FLAGS) $(TESTS)/lidarpcc_compress.cpp -fPIC -o $@

$(OBJ)/lidarpcc_decompress.o : $(TESTS)/lidarpcc_decompress.cpp \
		$(COMMON)/CLI11.hpp $(COMMON)/utils.h \
		$(COMMON)/file_io.h $(COMMON)/types.h \
		$(COMPRESS)/adaptive_compressor.h
	$(CC) -c $(FLAGS) $(TESTS)/lidarpcc_decompress.cpp -fPIC -o $@

#################### library ####################

$(BUILD)/libDBGC.so : $(OBJ)/dbscan_core.o \
		$(OBJ)/adaptive_compressor.o $(OBJ)/sline_compressor.o \
		$(OBJ)/quadtree.o $(OBJ)/octree.o \
		$(OBJ)/entropy.o
	$(CC) $(FLAGS) $(OBJ)/dbscan_core.o \
		$(OBJ)/adaptive_compressor.o $(OBJ)/sline_compressor.o \
		$(OBJ)/quadtree.o $(OBJ)/octree.o \
		$(OBJ)/entropy.o -shared -o $@ -lbz2 -llzma -lz -ltbb

#################### partition ####################

$(OBJ)/dbscan_core.o : $(PARTITION)/dbscan_core.cpp \
		$(COMMON)/config.h $(COMMON)/index.h
	$(CC) -c $(FLAGS) $(PARTITION)/dbscan_core.cpp -fPIC -o $@

#################### compressor ####################

$(OBJ)/adaptive_compressor.o : $(COMPRESS)/adaptive_compressor.cpp \
		$(COMMON)/types.h $(COMMON)/file_io.h \
		$(COMPRESS)/base.h \
		$(COMPRESS)/general.h \
		$(COMPRESS)/trees/octree.h \
		$(COMPRESS)/trees/quadtree.h \
		$(COMPRESS)/sline_compressor.h \
		$(COMPRESS)/adaptive_compressor.h \
		$(PARTITION)/by_lambda.h \
		$(PARTITION)/by_density.h
	$(CC) -c $(FLAGS) $(COMPRESS)/adaptive_compressor.cpp -fPIC -o $@

$(OBJ)/sline_compressor.o : $(COMPRESS)/sline_compressor.cpp \
		$(COMMON)/config.h $(COMMON)/types.h \
		$(COMMON)/index.h \
		$(COMMON)/polylines.h \
		$(COMPRESS)/base.h \
		$(COMPRESS)/general.h \
		$(COMPRESS)/entropy.h \
		$(COMPRESS)/sline_compressor.h \
		$(COMPRESS)/general.h
	$(CC) -c $(FLAGS) $(COMPRESS)/sline_compressor.cpp -fPIC -o $@

$(OBJ)/quadtree.o : $(COMPRESS)/trees/quadtree.cpp \
		$(COMMON)/types.h $(COMPRESS)/base.h \
		$(COMPRESS)/entropy.h \
		$(COMPRESS)/trees/quadtree.h
	$(CC) -c $(FLAGS) $(COMPRESS)/trees/quadtree.cpp -fPIC -o $@

$(OBJ)/octree.o : $(COMPRESS)/trees/octree.cpp \
		$(COMMON)/config.h $(COMMON)/types.h \
		$(COMPRESS)/base.h \
		$(COMPRESS)/entropy.h \
		$(COMPRESS)/trees/octree.h
	$(CC) -c $(FLAGS) $(COMPRESS)/trees/octree.cpp -fPIC -o $@

$(OBJ)/entropy.o : $(COMPRESS)/entropy.cpp \
		$(COMPRESS)/base.h $(COMPRESS)/entropy.h
	$(CC) -c $(FLAGS) $(COMPRESS)/entropy.cpp -fPIC -o $@

#################### end ####################


.PHONY: clean

clean: 
	rm -r ${BUILD}
