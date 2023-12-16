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
#include "disk.h"
#include "inode.h"
#include "util.h"

typedef struct
{
  Disk *disk;
  Inode *inode;
  unsigned int blocksize;
  unsigned int lastByteRead;
  const char *path;
  unsigned int fd;
} File;

//Funcao para instalar seu sistema de arquivos no S.O., registrando-o junto
//ao virtual FS (vfs). Retorna um identificador unico (slot), caso
//o sistema de arquivos tenha sido registrado com sucesso.
//Caso contrario, retorna -1
int installMyFS ( void );

#endif
