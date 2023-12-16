/*
*  myfs.h - Funcao que permite a instalacao de seu sistema de arquivos no S.O.
*
*  Autor: SUPER_PROGRAMADORES C
*  Projeto: Trabalho Pratico II - Sistemas Operacionais
*  Organizacao: Universidade Federal de Juiz de Fora
*  Departamento: Dep. Ciencia da Computacao
*
*
*/

#ifndef MYFS_H
#define MYFS_H

#include "vfs.h"
#include "inode.h"
#include "disk.h"

// Declaração da estrutura dos File Descriptors
// typedef struct fd {
//     int status;
//     int type;
//     char path[MAX_FILENAME_LENGTH + 1];
// } FD;

// typedef struct
// {
//   Disk *disk;
//   Inode *inode;
//   unsigned int blocksize;
//   unsigned int lastByteRead;
//   const char *path;
//   unsigned int fd;
// } File;

typedef struct 
{
    char filename[100];
    unsigned int inumber;
} LinkDir;

typedef struct
{
  Disk *disk;
  Inode *inode;
  unsigned int blocksize;
  unsigned int lastByteRead;
  const char *path;
  unsigned int fd;
  LinkDir link;
} File;

typedef struct MyFDFILE {
    int status;
    int type;
    char path[MAX_FILENAME_LENGTH + 1];
} MyFDFILE;

//Funcao para instalar seu sistema de arquivos no S.O., registrando-o junto
//ao virtual FS (vfs). Retorna um identificador unico (slot), caso
//o sistema de arquivos tenha sido registrado com sucesso.
//Caso contrario, retorna -1
int installMyFS ( void );

#endif
