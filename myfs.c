/*
*  myfs.c - Implementacao do sistema de arquivos MyFS
*
*  Autores: Ana Beatriz Lana M. M. Armond (202165501B)
            Davi Kirchmaier ( )
            Gabriella Cruz e Silva ( )
            Rayssa Amaral ( )
*  Projeto: Trabalho Pratico II - Sistemas Operacionais
*  Organizacao: Universidade Federal de Juiz de Fora
*  Departamento: Dep. Ciencia da Computacao
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "myfs.h"
#include "vfs.h"
#include "inode.h"
#include "util.h"

// Declarações Globais

// struct filedescriptor
// {
//   int status;
//   int type;
//   unsigned int fd;
//   unsigned int pointer;
//   Disk *disk;
//   Inode *inode;
//   const char *path;
// }; typedef struct filedescriptor FileDescriptor;
// FileDescriptor *fileDescriptors[MAX_FDS];

char fsid = 5;
char *fsname = "Matrix";

int myFSslot;
File *files[MAX_FDS] = {NULL};

// Variáveis Globais

#define SUPER_NUM_BLOCKS (3 * sizeof(unsigned int) + sizeof(char))
#define SUPER_BLOCKSIZE 0
#define SUPER_FREE_SPACE_SECTOR (sizeof(unsigned int) + sizeof(char))
#define SUPER_FIRST_BLOCK_SECTOR (2 * sizeof(unsigned int) + sizeof(char))

#define MAX_INODES_COUNT 1024
#define MAX_FILENAME_SIZE 255
#define MAX_DIR_ENTRIES 1024
#define MAX_OPEN_FILE_LIMIT 1024
Disk *atualDisk;

// Structs

typedef struct {
  Inode *inodes[MAX_INODES_COUNT];
  int used[MAX_INODES_COUNT];
} Inodes;
Inodes inodes;

typedef struct {
  char fileName[MAX_FILENAME_SIZE];
  unsigned int inodeNumber;
} DirectoryEntry;

typedef struct {
  DirectoryEntry entries[MAX_DIR_ENTRIES];
  int numberOfEntries;
} Directory;
Directory root;

typedef struct {
  int status; // 1 = arquivo aberto / 0 = arquivo fechado
  int type;
  unsigned int fd;
  unsigned int pointer;
  Disk *disk;
  Inode *inode;
  const char *path;
} FileDescriptor;
FileDescriptor fileDescriptors[MAX_FDS];

struct File
{
  Disk *disk;
  Inode *inode;
  unsigned int blocksize;
  unsigned int lastByteRead;
  const char *path;
  unsigned int fd;
};


// _______________________________ Funções Auxiliares _______________________________ //

int addDirectoryEntry(Directory *dir, const char *path, unsigned int numInode) {
  if (dir == NULL || path == NULL || dir->numberOfEntries >= MAX_DIR_ENTRIES) {
    return -1; // Parâmetros inválidos ou diretório cheio
  }

  strcpy(dir->entries[dir->numberOfEntries].fileName, path);
  dir->entries[dir->numberOfEntries].inodeNumber = numInode;
  dir->numberOfEntries++;

  return 0; // Sucesso
}

Inode *createInode(Disk *d) {
  if (d == NULL) {
    return NULL; // Parâmetro inválido
  }

  Inode *newInode = inodeCreate(inodeFindFreeInode(0, d), d);
  if (newInode == NULL) {
    return NULL; // Falha ao criar novo inode
  }

  return newInode;
}

// ___________________________ Fim das Funções Auxiliares ___________________________ //
 

//Funcao para verificacao se o sistema de arquivos está ocioso, ou seja,
//se nao ha quisquer descritores de arquivos em uso atualmente. Retorna
//um positivo se ocioso ou, caso contrario, 0.
int myFSIsIdle (Disk *d) {
	for (int i = 0; i < MAX_FDS; i++)
	{
		if (files[i] != NULL && diskGetId(d) == diskGetId(files[i]->disk))
		{
			return 0;
		}
	}
	return 1;
}

//Funcao para formatacao de um disco com o novo sistema de arquivos
//com tamanho de blocos igual a blockSize. Retorna o numero total de
//blocos disponiveis no disco, se formatado com sucesso. Caso contrario,
//retorna -1.
int myFSFormat(Disk *d, unsigned int blockSize) {
    unsigned char superblock[DISK_SECTORDATASIZE] = {0};

    ul2char(blockSize, &superblock[SUPER_BLOCKSIZE]); // Conversão de um unsigned int para um array de bytes

    unsigned int numInodes = (diskGetSize(d) / blockSize ) / 8;

    unsigned int freeSpaceSector = inodeAreaBeginSector() + numInodes / inodeNumInodesPerSector();
    unsigned int freeSpaceSize = (diskGetSize(d) / blockSize) / (sizeof(unsigned char) * 8 * DISK_SECTORDATASIZE);

    ul2char(freeSpaceSector, &superblock[SUPER_FREE_SPACE_SECTOR]);

    unsigned int firstBlockSector = freeSpaceSector + freeSpaceSize;
    unsigned int numBlocks = (diskGetNumSectors(d) - firstBlockSector) / (blockSize / DISK_SECTORDATASIZE);

    ul2char(firstBlockSector, &superblock[SUPER_FIRST_BLOCK_SECTOR]);
    ul2char(numBlocks, &superblock[SUPER_NUM_BLOCKS]);

    if(diskWriteSector(d, 0, superblock) == -1)
    {
        return -1;
    }
      
    unsigned char freeSpace[DISK_SECTORDATASIZE] = {0};
      for(int i=0; i<freeSpaceSize ; i++)
      {
        if(diskWriteSector(d, freeSpaceSector + i, freeSpace) == -1)
        {
          return -1;
        }
      }

      return numBlocks > 0 ? numBlocks : -1;
}

//Funcao para abertura de um arquivo, a partir do caminho especificado
//em path, no disco montado especificado em d, no modo Read/Write,
//criando o arquivo se nao existir. Retorna um descritor de arquivo,
//em caso de sucesso. Retorna -1, caso contrario.
int myFSOpen (Disk *d, const char *path) {
  if (d == NULL || path == NULL) 
  {
      return -1; // Argumentos inválidos
  }
  atualDisk = d; 

	// Procura se o arquivo já existe
	int fileIndex = -1;
	for (int i = 0; i < root.numberOfEntries; i++)
	{
		if (strcmp(root.entries[i].fileName, path) == 0)
		{
			fileIndex = i;
			break;
		}
	}

	if (fileIndex == -1)
	{
		// O arquivo não existe, então cria um novo
		Inode *newInode = createInode(d);
		if (newInode == NULL)
		{
			return -1; // Falha ao criar um novo inode
		}

		int numInode = inodeGetNumber(newInode);

		if (addDirectoryEntry(&root, path, numInode) == -1)
		{
			return -1; // Falha ao adicionar a entrada no diretório
		}

		fileIndex = root.numberOfEntries - 1;
	}

	// Encontra um descritor de arquivo livre
	for (int i = 1; i < MAX_FDS - 1; i++)
	{
		if (!fileDescriptors[i].status)
		{
			fileDescriptors[i].inode = inodeLoad(root.entries[fileIndex].inodeNumber, d);
			fileDescriptors[i].status = 1;
			return i; // Retorna o índice do descritor de arquivo
		}
	}

	return -1; // Todos os descritores de arquivo estão em uso
}
	
//Funcao para a leitura de um arquivo, a partir de um descritor de
//arquivo existente. Os dados lidos sao copiados para buf e terao
//tamanho maximo de nbytes. Retorna o numero de bytes efetivamente
//lidos em caso de sucesso ou -1, caso contrario.
int myFSRead (int fd, char *buf, unsigned int nbytes) {
	if(fd < 0 || fd >= MAX_FDS) 
  {
    return -1;
  }

  File* file = files[fd];
  if(file == NULL)
  {
    return -1;
  }
      
  unsigned int fileSize = inodeGetFileSize(file->inode);
  unsigned int bytesRead = 0;
  unsigned int currentInodeBlockNumber = file->lastByteRead / file->blocksize;
  unsigned int offset = file->lastByteRead % file->blocksize;
  unsigned int currentBlock = inodeGetBlockAddr(file->inode, currentInodeBlockNumber);
  unsigned char diskBuffer[DISK_SECTORDATASIZE];

    while(bytesRead < nbytes && bytesRead + file->lastByteRead < fileSize && currentBlock > 0) 
    {
        unsigned int sectorsPerBlock = file->blocksize / DISK_SECTORDATASIZE;
        unsigned int firstSector = offset / DISK_SECTORDATASIZE;
        unsigned int firstByteInSector = offset % DISK_SECTORDATASIZE;

        for(int i = firstSector; i < sectorsPerBlock && bytesRead < nbytes; i++) 
        {
            if(diskReadSector(file->disk, currentBlock + i, diskBuffer) == -1)
                return -1;

            for(int j = firstByteInSector;  j < DISK_SECTORDATASIZE && bytesRead < nbytes && bytesRead + file->lastByteRead < fileSize;  j++) 
            {
                buf[bytesRead] = diskBuffer[j];
                bytesRead++;
            }

            firstByteInSector = 0;
        }

        offset = 0;
        currentInodeBlockNumber++;
        currentBlock = inodeGetBlockAddr(file->inode, currentInodeBlockNumber);
    }

    file->lastByteRead += bytesRead;

    return bytesRead;
}

//Funcao para a escrita de um arquivo, a partir de um descritor de
//arquivo existente. Os dados de buf serao copiados para o disco e
//terao tamanho maximo de nbytes. Retorna o numero de bytes
//efetivamente escritos em caso de sucesso ou -1, caso contrario
int myFSWrite (int fd, const char *buf, unsigned int nbytes) {
	//  if(fileDescriptors[fd] == NULL || fileDescriptors[fd]->status == 0)
  //  {
  //     // O arquivo não está aberto
  //     return -1;
  //  }
    
}

//Funcao para fechar um arquivo, a partir de um descritor de arquivo
//existente. Retorna 0 caso bem sucedido, ou -1 caso contrario
int myFSClose (int fd) {
  // if(fd <= 0 || fd > MAX_FDS)
  // {
  //   return -1;
  // }
  // if(fileDescriptors[fd] == NULL || fileDescriptors[fd]->status == 0)
  // {
  //   // Já está fechado
  //   return -1;
  // }

  File *file = files[fd];

  if(!file)
  {
    return -1;
  }

  files[fd - 1] = NULL;
  free(file->inode);
  free(file);

  return 0;
}

//Funcao para abertura de um diretorio, a partir do caminho
//especificado em path, no disco indicado por d, no modo Read/Write,
//criando o diretorio se nao existir. Retorna um descritor de arquivo,
//em caso de sucesso. Retorna -1, caso contrario.
// int myFSOpenDir (Disk *d, const char *path) {
// 	return -1;
// }

//Funcao para a leitura de um diretorio, identificado por um descritor
//de arquivo existente. Os dados lidos correspondem a uma entrada de
//diretorio na posicao atual do cursor no diretorio. O nome da entrada
//e' copiado para filename, como uma string terminada em \0 (max 255+1).
//O numero do inode correspondente 'a entrada e' copiado para inumber.
//Retorna 1 se uma entrada foi lida, 0 se fim de diretorio ou -1 caso
//mal sucedido
// int myFSReadDir (int fd, char *filename, unsigned int *inumber) {
// 	return -1;
// }

//Funcao para adicionar uma entrada a um diretorio, identificado por um
//descritor de arquivo existente. A nova entrada tera' o nome indicado
//por filename e apontara' para o numero de i-node indicado por inumber.
//Retorna 0 caso bem sucedido, ou -1 caso contrario.
// int myFSLink (int fd, const char *filename, unsigned int inumber) {
// 	return -1;
// }

//Funcao para remover uma entrada existente em um diretorio, 
//identificado por um descritor de arquivo existente. A entrada e'
//identificada pelo nome indicado em filename. Retorna 0 caso bem
//sucedido, ou -1 caso contrario.
// int myFSUnlink (int fd, const char *filename) {
// 	return -1;
// }

//Funcao para fechar um diretorio, identificado por um descritor de
//arquivo existente. Retorna 0 caso bem sucedido, ou -1 caso contrario.	
// int myFSCloseDir (int fd) {
// 	return -1;
// }

//Funcao para instalar seu sistema de arquivos no S.O., registrando-o junto
//ao virtual FS (vfs). Retorna um identificador unico (slot), caso
//o sistema de arquivos tenha sido registrado com sucesso.
//Caso contrario, retorna -1
int installMyFS (void) {
	// FSInfo *fs_info = (FSInfo*)malloc(sizeof(FSInfo));
  //   fs_info->fsname = fsname;
  //   fs_info->fsid = fsid;
  //   fs_info->closeFn = myFSClose;
  //   fs_info->formatFn = myFSFormat;
  //   fs_info->isidleFn = myFSIsIdle;
  //   fs_info->openFn = myFSOpen;
  //   fs_info->readFn = myFSRead;
  //   fs_info->writeFn = myFSWrite;
  //   myFSslot = vfsRegisterFS(fs_info);
  //   return myFSslot;
  FSInfo *fs_info = (FSInfo *)malloc(sizeof(FSInfo));
    fs_info->fsname = fsname;
    fs_info->fsid = fsid;
    fs_info->closeFn = myFSClose;
    fs_info->formatFn = myFSFormat;
    fs_info->isidleFn = myFSIsIdle;
    fs_info->openFn = myFSOpen;
    fs_info->readFn = myFSRead;
    fs_info->writeFn = myFSWrite;
    myFSslot = vfsRegisterFS(fs_info);
    return myFSslot;
}
