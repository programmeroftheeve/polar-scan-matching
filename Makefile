#there are three targets: psm library, test program and an example program.
OUTPUT	=	obj
DIR    =  polar_matching
DIR_TEST	=	test
LIB	=	psm
PRG    =  run_test
PRG_EXAMPLE	= example
OBJ_LIB    =  $(OUTPUT)/draw.o $(OUTPUT)/polar_match.o  
OBJ_MAIN	=	$(OUTPUT)/main.o
OBJ_EXAMPLE    =  $(OUTPUT)/example.o
LINK   =  -lstdc++ -lX11 -lm -L/usr/lib -L/lib -l$(LIB) -lrt -L. # -pg #uncomment -pg for profiling.
C_OPTS =  -c -g -Wall -I$(DIR) #-O2 # -pg #if not debugging uncomment the optimization switch -O2

lib$(LIB).a : $(OBJ_LIB) 
	ar r lib$(LIB).a $(OBJ_LIB)

$(PRG) :	$(OBJ_MAIN)	lib$(LIB).a
	gcc -o  $(PRG) $(OBJ_MAIN) $(LINK)

$(PRG_EXAMPLE) :	$(OBJ_EXAMPLE)	lib$(LIB).a
	gcc -o  $(PRG_EXAMPLE) $(OBJ_EXAMPLE) $(LINK)

$(OUTPUT)/example.o : $(DIR_TEST)/example.cpp $(DIR)/draw.h $(DIR)/polar_match.h
	gcc  $(C_OPTS) $(DIR_TEST)/example.cpp -o $(OUTPUT)/example.o -Ipolar_matching
 
$(OUTPUT)/main.o : $(DIR_TEST)/main.cpp $(DIR)/draw.h $(DIR)/polar_match.h
	gcc  $(C_OPTS) $(DIR_TEST)/main.cpp -o $(OUTPUT)/main.o
 
$(OUTPUT)/draw.o : $(DIR)/draw.h $(DIR)/draw.c
	gcc $(C_OPTS) $(DIR)/draw.c -o $(OUTPUT)/draw.o

$(OUTPUT)/polar_match.o :  $(DIR)/polar_match.cpp $(DIR)/polar_match.h $(DIR)/draw.h
	gcc $(C_OPTS) $(DIR)/polar_match.cpp -o $(OUTPUT)/polar_match.o

all	:	lib$(LIB).a	$(PRG) $(PRG_EXAMPLE)
  
clean :
	rm lib$(LIB).a $(PRG_EXAMPLE) $(PRG) $(OUTPUT)/* $(EXMPL_PRG) *.tag *~ */*~ tags
 
zip :
	zip -r ../$(DIR).zip *
