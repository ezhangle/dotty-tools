CC=gcc

# suggested compiler-specific flags
CLANGFLAGS=	-ansi -pedantic --analyze
GCCFLAGS=	-ansi -pedantic -Wall -Wextra -O2
TCCFLAGS=	-Wall -Wunsupported
PCCFLAGS=	

# the compiler flags actually used
CFLAGS=$(GCCFLAGS)

PCL_CFLAGS=-I/usr/include/pcl-1.6 -I/usr/include/eigen3

TGT_DIR=$(HOME)/bin
INST_DIR=/usr/local/bin

all:	gen_analytic \
	pca-pcl \
	scale \
	ysort \
	data_size \
	bnpts2npts \
	noff2npts \
	swap_yz \
	shift_data \
	evec_angles \
	rotn_comp \
	apply_rotation.c

clean:
	rm utilities.o
	rm -f $(TGT_DIR)/evec_angles
	rm -f $(TGT_DIR)/gen_analytic
	rm -f $(TGT_DIR)/scale
	rm -f $(TGT_DIR)/ysort
	rm -f $(TGT_DIR)/data_size
	rm -f $(TGT_DIR)/bnpts2npts
	rm -f $(TGT_DIR)/noff2npts
	rm -f $(TGT_DIR)/swap_yz
	rm -f $(TGT_DIR)/shift_data
	rm -f $(TGT_DIR)/pca-pcl
	rm -f $(TGT_DIR)/rotn_comp
	rm -f $(TGT_DIR)/apply_rotation

install:
	make all
	cp -f $(TGT_DIR)/gen_analytic	$(INST_DIR)/
	cp -f $(TGT_DIR)/scale		$(INST_DIR)/
	cp -f $(TGT_DIR)/ysort		$(INST_DIR)/
	cp -f $(TGT_DIR)/data_size	$(INST_DIR)/
	cp -f $(TGT_DIR)/bnpts2npts	$(INST_DIR)/
	cp -f $(TGT_DIR)/noff2npts	$(INST_DIR)/
	cp -f $(TGT_DIR)/swap_yz	$(INST_DIR)/
	cp -f $(TGT_DIR)/shift_data	$(INST_DIR)/
	cp -f $(TGT_DIR)/evec_angles	$(INST_DIR)/
	cp -f $(TGT_DIR)/pca-pcl	$(INST_DIR)/
	cp -f $(TGT_DIR)/rotn_comp	$(INST_DIR)/
	cp -f $(TGT_DIR)/apply_rotation	$(INST_DIR)/

uninstall:
	rm -f $(INST_DIR)/gen_analytic
	rm -f $(INST_DIR)/scale
	rm -f $(INST_DIR)/ysort
	rm -f $(INST_DIR)/data_size
	rm -f $(INST_DIR)/bnpts2npts
	rm -f $(INST_DIR)/noff2npts
	rm -f $(INST_DIR)/swap_yz
	rm -f $(INST_DIR)/shift_data
	rm -f $(INST_DIR)/pca-pcl
	rm -f $(INST_DIR)/rotn_comp
	rm -f $(INST_DIR)/apply_rotation

evec_angles:	evec_angles.c
	$(CC) $(CFLAGS) $(<) -o $(TGT_DIR)/$(@) -lm

rotn_comp:	rotn_comp.c
	$(CC) $(CFLAGS) $(<) -o $(TGT_DIR)/$(@) -lm

apply_rotation:	apply_rotation.c
	$(CC) $(CFLAGS) $(<) -o $(TGT_DIR)/$(@) -lm

pca-pcl:	pca-pcl.cpp
	g++ $(PCL_CFLAGS) $(<) utilities.o -o $(TGT_DIR)/$(@) -lpcl_common

ysort:		ysort.c utilities.o
	$(CC) $(CFLAGS) -lm $(<) utilities.o -o $(TGT_DIR)/$(@)

shift_data:	shift_data.c utilities.o
	$(CC) $(CFLAGS) $(<) utilities.o -o $(TGT_DIR)/$(@)

swap_yz:	swap_yz.c utilities.o
	$(CC) $(CFLAGS) $(<) utilities.o -o $(TGT_DIR)/$(@)

gen_analytic:	gen_analytic.c utilities.o
	$(CC) $(CFLAGS) $(<) utilities.o -o $(TGT_DIR)/$(@) -lm

scale:	scale.c utilities.o
	$(CC) $(CFLAGS) $(<) utilities.o -o $(TGT_DIR)/$(@)

data_size:	data_size.c utilities.o
	$(CC) $(CFLAGS) $(<) utilities.o -o $(TGT_DIR)/$(@)

noff2npts:	noff2npts.c utilities.o
	$(CC) $(CFLAGS) $(<) utilities.o -o $(TGT_DIR)/$(@)

bnpts2npts:	bnpts2npts.c utilities.o
	$(CC) $(CFLAGS) $(<) utilities.o -o $(TGT_DIR)/$(@)

utilities.o:	utilities.c utilities.h
	$(CC) -c $(CFLAGS) $(<) -o $(@)

