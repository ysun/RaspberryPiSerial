CC := gcc 
CFLAGS := -Wall -O -g

TARGET = control  #This is the binary name!
SOURCE = $(wildcard *.c)  
OBJS = $(patsubst %.c,%.o,$(SOURCE))  

$(TARGET) : $(OBJS)
	$(CC) $^ -o $@

%.o : %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	-rm $(TARGET) $(OBJS)


###############################################
# TARGET:=control
# CC:=gcc
# FILES=`ls *.c`
# 
# all: $(TARGET)
# 
# $(TARGET): *.o
# 	$(CC) -o $@ $^
# 
# clean:
# 	-rm $(TARGET) *.o
# 
# define compile_c_file
# @for file in $(FILES); do \
# 	( echo "$(CC) -c $$file" && $(CC) -c $$file ) \
# 	done;
# endef
# 
# %.o: %.c
# 	$(call compile_c_file)
