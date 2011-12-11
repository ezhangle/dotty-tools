CC=gcc

# suggested compiler-specific flags
CLANGFLAGS=	-ansi -pedantic --analyze
GCCFLAGS=	-ansi -pedantic -Wall -Wextra -O2
TCCFLAGS=	-Wall -Wunsupported
PCCFLAGS=	

# the compiler flags actually used
CFLAGS=$(GCCFLAGS)

# linking flags used for the principal components analysis tool
PCA_LD_FLAGS=-L/usr/local/lib -uloclib -lgsl -lgslcblas
PCA_FLAGS=$(CFLAGS) -ulocinclude -I/usr/local/include

TGT_DIR=$(HOME)/bin
INST_DIR=/usr/local/bin

all:	gen_analytic \
	scale \
	ysort \
	data_size \
	bnpts2npts \
	noff2npts \
	swap_yz \
	shift_data

clean:
	rm -f $(TGT_DIR)/gen_analytic
	rm -f $(TGT_DIR)/scale
	rm -f $(TGT_DIR)/ysort
	rm -f $(TGT_DIR)/data_size
	rm -f $(TGT_DIR)/bnpts2npts
	rm -f $(TGT_DIR)/noff2npts
	rm -f $(TGT_DIR)/swap_yz
	rm -f $(TGT_DIR)/shift_data

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

uninstall:
	rm -f $(INST_DIR)/gen_analytic
	rm -f $(INST_DIR)/scale
	rm -f $(INST_DIR)/ysort
	rm -f $(INST_DIR)/data_size
	rm -f $(INST_DIR)/bnpts2npts
	rm -f $(INST_DIR)/noff2npts
	rm -f $(INST_DIR)/swap_yz
	rm -f $(INST_DIR)/shift_data

pca:		pca.c pca.h 
	$(CC) $(PCA_FLAGS) -o $(TGT_DIR)/pca pca.c $(PCA_LD_FLAGS) 

ysort:		ysort.c
	$(CC) $(CFLAGS) -lm $(<) -o $(TGT_DIR)/$(@)

shift_data:	shift_data.c
	$(CC) $(CFLAGS) $(<) -o $(TGT_DIR)/$(@)

swap_yz:	swap_yz.c
	$(CC) $(CFLAGS) $(<) -o $(TGT_DIR)/$(@)

gen_sphere:	gen_sphere.c
	$(CC) $(CFLAGS) -lm $(<) -o $(TGT_DIR)/$(@)

gen_analytic:	gen_analytic.c utilities.o
	$(CC) $(CFLAGS) -lm $(<) utilities.o -o $(TGT_DIR)/$(@)

scale:	scale.c utilities.o
	$(CC) $(CFLAGS) $(<) utilities.o -o $(TGT_DIR)/$(@)

data_size:	data_size.c utilities.o
	$(CC) $(CFLAGS) $(<) utilities.o -o $(TGT_DIR)/$(@)

noff2npts:	noff2npts.c
	$(CC) $(CFLAGS) $(<) -o $(TGT_DIR)/$(@)

bnpts2npts:	bnpts2npts.c
	$(CC) $(CFLAGS) $(<) -o $(TGT_DIR)/$(@)

utilities.o:	utilities.c utilities.h
	$(CC) -c $(CFLAGS) $(<) -o $(@)

