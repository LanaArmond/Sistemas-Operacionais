/*
*  myfs.c - Implementacao do sistema de arquivos MyFS
*
*  Autores: * Ana Beatriz Lana Maciel Moreira Armond (202165501B)
*             Davi Kirchmaier Paiva (202176021)
*             Gabriella Cruz e Silva (202165512N)
*             Rayssa Amaral Gomes (202265194AB)
*  Projeto: Trabalho Pratico II - Sistemas Operacionais
*  Organizacao: Universidade Federal de Juiz de Fora
*  Departamento: Dep. Ciencia da Computacao
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "myfs.h"
#include "vfs.h"
#include "inode.h"
#include "util.h"

/// Struct File
typedef struct
{
  Disk *disk;   // Ponteiro para a estrutura Disk associada ao arquivo
  Inode *inode;   // Ponteiro para a estrutura Inode associada ao arquivo
  unsigned int blocksize;   // Tamanho do bloco do arquivo
  unsigned int lastByteRead;   // Último byte lido no arquivo
  const char *path;   // Caminho do arquivo
  unsigned int fd;   // Descritor de arquivo (file descriptor)
} File;

// Nome e ID do Sistema Operacional
// char fsid = 5;
// char *fsname = "Matrix";

// Variáveis Globais

// Tamanho do setor de superbloco em bytes
#define SUPER_NUM_BLOCKS (3 * sizeof(unsigned int) + sizeof(char))
// Índice do byte que armazena o tamanho do bloco no superbloco
#define SUPER_BLOCKSIZE 0
// Índice do byte que armazena o identificador do sistema de arquivos (FSID) no superbloco
#define SUPERBLOCK_FSID sizeof(unsigned int)
// Índice do byte que armazena o setor onde começa o espaço livre no superbloco
#define SUPER_FREE_SPACE_SECTOR (sizeof(unsigned int) + sizeof(char))
// Índice do byte que armazena o setor onde começa o primeiro bloco de dados no superbloco
#define SUPER_FIRST_BLOCK_SECTOR (2 * sizeof(unsigned int) + sizeof(char))
// Número do inode do diretório raiz no sistema de arquivos
#define ROOT_DIRECTORY_INODE 1

// Declarações Globais
FSInfo *fsInfo;
File *files[128];
int myFSslot;
// Disk* atualDisk;

// _______________________________ Funções Auxiliares _______________________________ //

// Retorna a posição do primeiro bit zero encontrado na representação binária de um byte
int firstBitZero(unsigned char byte)
{
	unsigned char mask = 1;
  // Percorre cada bit do byte para encontrar o primeiro bit zero
	for (int i = 0; i < sizeof(unsigned char); i++)
	{
		if ((mask & byte) == 0)
    {
      return i; // Retorna a posição do primeiro bit zero
    }
		mask <<= (unsigned char)1;
	}

	return -1; // Retorna -1 se não houver bits zero no byte
}

// Define o bit na posição especificada para 1 no byte dado
unsigned char setBitToOne(unsigned char byte, unsigned int bit)
{
	unsigned char mask = (unsigned char)1 << bit; // Cria uma máscara com um bit na posição desejada
	return byte | mask; // Realiza uma operação OR para definir o bit para 1 no byte
}

// Define o bit na posição especificada para 0 no byte dado
unsigned char setBitToZero(unsigned char byte, unsigned int bit)
{
	unsigned char mask = ((unsigned char)1 << bit); // Cria uma máscara com um bit na posição desejada
	mask = ~mask; // Inverte a máscara para ter 0 no bit desejado
	return byte & mask; // Realiza uma operação AND para definir o bit para 0 no byte
}

// Função para encontrar um bloco livre no disco
unsigned int findFreeBlock(Disk *disk)
{
  // Buffer para armazenar dados do setor do disco
	unsigned char buffer[DISK_SECTORDATASIZE];

  // Lê o setor superblock para obter informações essenciais
	if (diskReadSector(disk, 0, buffer) == -1)
  {
    return -1; // Retorna -1 em caso de erro na leitura do setor
  }
	
  // Obtém o número de setores por bloco do superblock
	unsigned int sectorsPerBlock;
	char2ul(&buffer[SUPER_BLOCKSIZE], &sectorsPerBlock);
	sectorsPerBlock /= DISK_SECTORDATASIZE;

  // Obtém o número total de blocos do superblock
	unsigned int numberOfBlocks;
	char2ul(&buffer[SUPER_NUM_BLOCKS], &numberOfBlocks);

  // Obtém o setor do primeiro bloco do superblock
	unsigned int firstBlock;
	char2ul(&buffer[SUPER_FIRST_BLOCK_SECTOR], &firstBlock);

  // Obtém o setor do espaço livre do superblock
	unsigned int freeSpaceSector;
	char2ul(&buffer[SUPER_FREE_SPACE_SECTOR], &freeSpaceSector);

  // Calcula o tamanho do espaço livre em setores
	unsigned int freeSpaceSize = firstBlock - freeSpaceSector;

  // Itera sobre os setores do espaço livre
	for (int i = freeSpaceSector; i < freeSpaceSector + freeSpaceSize; i++)
	{
    // Lê o setor atual do espaço livre
		if (diskReadSector(disk, i, buffer) == -1)
    {
      return -1; // Retorna -1 em caso de erro na leitura do setor
    }
		
    // Itera sobre os bytes do setor
		for (int j = 0; j < DISK_SECTORDATASIZE; j++)
		{
      // Encontra o primeiro bit zero no byte
			int freeBit = firstBitZero(buffer[j]);

      // Verifica se um bit zero foi encontrado
			if (freeBit != -1)
			{
        // Calcula o número do bloco livre
				unsigned int freeBlock = firstBlock + (i - freeSpaceSector) * DISK_SECTORDATASIZE * 8 * sectorsPerBlock + j * 8 * sectorsPerBlock + freeBit * sectorsPerBlock;

        // Verifica se o bloco está além do número total de blocos
				if ((freeBlock - firstBlock) / sectorsPerBlock >= numberOfBlocks)
        {
          return -1; // Retorna -1 se o bloco estiver além do número total de blocos
        }
				
        // Define o bit correspondente a 1
				buffer[j] = setBitToOne(buffer[j], freeBit);

        // Escreve o setor atual de volta no disco
				if (diskWriteSector(disk, i, buffer) == -1)
        {
          return -1; // Retorna -1 em caso de erro na escrita do setor
        }

				return freeBlock; // Retorna o número do bloco livre encontrado
			}
		}
	}
	return -1; // Retorna -1 se nenhum bloco livre for encontrado
}

// Libera o bloco especificado no sistema de arquivos
bool setBlockFree(Disk *d, unsigned int block)
{
  // Buffer para armazenar dados lidos do disco
	unsigned char buffer[DISK_SECTORDATASIZE];

  // Lê o setor do superbloco
	if (diskReadSector(d, 0, buffer) == -1)
  {
    return false;
  }

  // Obtém informações do superbloco
	unsigned int sectorsPerBlock;
	char2ul(&buffer[SUPER_BLOCKSIZE], &sectorsPerBlock);
	sectorsPerBlock /= DISK_SECTORDATASIZE;

  // Obtém o número total de blocos no sistema de arquivos a partir do superbloco
	unsigned int numberOfBlocks;
	char2ul(&buffer[SUPER_NUM_BLOCKS], &numberOfBlocks);

  // Obtém o setor inicial do primeiro bloco a partir do superbloco
	unsigned int firstBlock;
	char2ul(&buffer[SUPER_FIRST_BLOCK_SECTOR], &firstBlock);

  // Obtém o setor inicial da área de espaço livre a partir do superbloco
	unsigned int freeSpaceStartSector;
	char2ul(&buffer[SUPER_FREE_SPACE_SECTOR], &freeSpaceStartSector);

  // Verifica se o bloco está além do limite de blocos do sistema de arquivos
	if ((block - firstBlock) / sectorsPerBlock >= numberOfBlocks)
  {
    return false;
  }
  
  // Calcula o setor de espaço livre correspondente ao bloco
	unsigned int blockFreeSpaceSector = ((block - firstBlock) / sectorsPerBlock) / (DISK_SECTORDATASIZE * 8);

  // Lê o setor de espaço livre correspondente ao bloco
	if (diskReadSector(d, blockFreeSpaceSector, buffer) == -1)
  {
    return false;
  }

  // Calcula o bit correspondente ao bloco no setor de espaço livre
	unsigned int blockFreeSpaceBit = ((block - firstBlock) / sectorsPerBlock) % (DISK_SECTORDATASIZE * 8);

  // Marca o bit como zero no setor de espaço livre
	buffer[blockFreeSpaceBit / 8] = setBitToZero(buffer[blockFreeSpaceBit / 8], blockFreeSpaceBit % 8);

  // Escreve de volta o setor de espaço livre no disco
	if (diskWriteSector(d, blockFreeSpaceSector, buffer) == -1)
  {
    return false;
  }
		
	return true;
}

// Função para obter o descritor de arquivo (File) associado a um caminho em um disco
File *getFile(Disk *d, const char *path)
{
  // Itera pelos file descriptors
	for (int i = 0; i < MAX_FDS; i++)
	{
    // Verifica se o descritor de arquivo na posição i é não nulo e pertence ao disco d e se o caminho do arquivo corresponde ao path fornecido
		if (files[i] != NULL && files[i]->disk == d && strcmp(files[i]->path, path) == 0)
		{
			return files[i]; // Retorna o descritor de arquivo associado ao caminho
		}
	}
	return NULL; // Retorna NULL se o arquivo não for encontrado
}

// ___________________________ Fim das Funções Auxiliares ___________________________ //

// Função para verificação se o sistema de arquivos está ocioso, ou seja, se não há quisquer descritores de arquivos em uso atualmente. 
// Retorna um positivo se ocioso ou, caso contrário, 0.
int myFSIsIdle(Disk *d)
{
  // A variável i anda por todos os file descriptors
	for (int i = 0; i < MAX_FDS; i++)
	{
    // Se o arquivo de file descriptor i NÃO for nulo e estiver em um disco ele NÃO está "idle"
		if (files[i] != NULL && diskGetId(d) == diskGetId(files[i]->disk))
		{
			return 0;
		}
	}
  // Caso contrário está "idle"
	return 1;
}

// Função para formatação de um disco com o novo sistema de arquivos com tamanho de blocos igual a blockSize. 
// Retorna o número total de blocos disponíveis no disco, se formatado com sucesso. Caso contrário, retorna -1.
int myFSFormat(Disk *d, unsigned int blockSize)
{
  // Inicializa um array de bytes para o superbloco
	unsigned char superblock[DISK_SECTORDATASIZE] = {0};

  // Converte o tamanho do bloco para bytes e armazena no superbloco
	ul2char(blockSize, &superblock[SUPER_BLOCKSIZE]); 

  // Armazena o identificador do sistema de arquivos no superbloco
	superblock[SUPERBLOCK_FSID] = fsInfo->fsid;

  // Calcula o número de inodes necessários
	unsigned int numInodes = (diskGetSize(d) / blockSize) / 8;

  // Cria inodes para cada bloco necessário
	for(int i=1; i<=numInodes; i++) 
  {
		Inode* inode = inodeCreate(i, d);
		if(inode == NULL)
    {
      return -1;
    }
		free(inode);
	}

  // Calcula o setor de início da área de espaço livre no disco
	unsigned int freeSpaceSector = inodeAreaBeginSector() + numInodes / inodeNumInodesPerSector();

  // Calcula o tamanho da área de espaço livre
	unsigned int freeSpaceSize = 1 + (diskGetSize(d) / blockSize) / (sizeof(unsigned char) * 8 * DISK_SECTORDATASIZE);

  // Converte o setor de início da área de espaço livre para bytes e armazena no superbloco
	ul2char(freeSpaceSector, &superblock[SUPER_FREE_SPACE_SECTOR]);

  // Calcula o setor de início do primeiro bloco de dados
	unsigned int firstBlockSector = freeSpaceSector + freeSpaceSize;

  // Calcula o número total de blocos no disco
	unsigned int numberOfBlocks = (diskGetNumSectors(d) - firstBlockSector) / (blockSize / DISK_SECTORDATASIZE);

  // Converte o setor de início do primeiro bloco e o número total de blocos para bytes e armazena no superbloco
	ul2char(firstBlockSector, &superblock[SUPER_FIRST_BLOCK_SECTOR]);
	ul2char(numberOfBlocks, &superblock[SUPER_NUM_BLOCKS]);

  // Escreve o superbloco no primeiro setor do disco
	if (diskWriteSector(d, 0, superblock) == -1)
  {
    return -1;
  }
		
  // Inicializa um array de bytes para a área de espaço livre
	unsigned char freeSpace[DISK_SECTORDATASIZE] = {0};

  // Preenche a área de espaço livre com setores vazios
	for (int i = 0; i < freeSpaceSize; i++)
	{
		if (diskWriteSector(d, freeSpaceSector + i, freeSpace) == -1)
		{
			return -1;
		}
	}

  // Carrega o inode do diretório raiz
	Inode* root = inodeLoad(ROOT_DIRECTORY_INODE, d);
	if(root == NULL)
  {
    return -1;
  }
		
	inodeSetFileSize(root, 0); // Configura o tamanho do arquivo do diretório raiz como zero
	inodeSetRefCount(root, 1); // Configura a contagem de referências do diretório raiz como 1
	inodeSetFileType(root, FILETYPE_REGULAR); // Configura o tipo de arquivo do diretório raiz como regular
	
	inodeSave(root); // Salva as alterações no inode do diretório raiz
	free(root); // Libera a memória do inode do diretório raiz

  // Retorna o número total de blocos se formatado com sucesso, caso contrário, retorna -1
	if (numberOfBlocks > 0)
  {
      return numberOfBlocks; // Sucesso
  }
  else
  {
      return -1; // Falha
  }
}

// Função para abertura de um arquivo, a partir do caminho especificado em path, no disco montado especificado em d, no modo Read/Write, criando o arquivo se não existir. 
// Retorna um descritor de arquivo, em caso de sucesso. Retorna -1, caso contrário.
int myFSOpen(Disk *d, const char *path)
{
  // Verifica se o arquivo já existe
	File *file = getFile(d, path);
	int inodeNumber;

  // Se o arquivo não existir, cria um novo
	if (file == NULL)
	{
		Inode *inode = NULL;

    // Procura por um slot vazio nos descritores de arquivo
		for (int i = 0; i < MAX_FDS; i++)
		{
			if (files[i] == NULL)
			{
        // Encontrou um slot vazio, então cria um novo inode
				inodeNumber = inodeFindFreeInode(1, d);
				inode = inodeCreate(inodeNumber, d);
				break;
			}
		}

    // Se não foi possível criar o inode, retorna erro
		if (inode == NULL)
    {
      return -1;
    }
			
    // Aloca memória para a estrutura File e os dados do arquivo
		file = malloc(sizeof(File));
		file->disk = d;
		file->path = path;
		file->inode = inode;
		file->fd = inodeGetNumber(inode);
    file->blocksize = 1024;   // OBS: dúvida sobre acessar o blocksize do disco
    file->lastByteRead = 0;   // Como ainda nada foi lido, alocamos 0

    // Armazena o novo arquivo no array de arquivos abertos
		files[file->fd-1] = file;
	}

  // Retorna o descritor de arquivo
	return file->fd;
}

// Função para a leitura de um arquivo, a partir de um descritor de arquivo existente. 
// Os dados lidos são copiados para buf e terão tamanho máximo de nbytes. 
// Retorna o número de bytes efetivamente lidos em caso de sucesso ou -1, caso contrário.
int myFSRead(int fd, char *buf, unsigned int nbytes)
{
  // Verifica se o descritor de arquivo é válido
	if (fd < 0 || fd >= MAX_FDS)
  {
    return -1;
  }
		
  // Obtém o arquivo correspondente ao descritor
	File *file = files[fd-1];
	if (file == NULL)
  {
    return -1;
  }
	
  // Obtém o tamanho total do arquivo
	unsigned int fileSize = inodeGetFileSize(file->inode);

  // Número de bytes já lidos no contexto atual
	unsigned int bytesRead = 0;

  // Calcula o número do bloco do inode que contém o byte atual do arquivo
	unsigned int currentInodeBlockNumber = file->lastByteRead / file->blocksize;

  // Calcula o offset dentro do bloco do inode para o byte atual do arquivo
	unsigned int offset = file->lastByteRead % file->blocksize;

  // Obtém o número do bloco no disco correspondente ao bloco do inode
	unsigned int currentBlock = inodeGetBlockAddr(file->inode, currentInodeBlockNumber);

  // Buffer para armazenar dados lidos do disco
	unsigned char diskBuffer[DISK_SECTORDATASIZE];

  //printf("bytesRead: %u, nbytes: %u, lastRead: %u, filesize: %u, currentBlock: %u", bytesRead, nbytes, file->lastByteRead, fileSize, currentBlock);
	
  // Condições de continuidade da leitura:
  // - Ainda há bytes a serem lidos (bytesRead < nbytes)
  // - Não ultrapassou o tamanho total do arquivo (bytesRead + file->lastByteRead < fileSize)
  // - O bloco atual no disco é válido (currentBlock > 0)
  while (bytesRead < nbytes && bytesRead + file->lastByteRead < fileSize && currentBlock > 0)
	{
    // Número de setores em um bloco do inode
		unsigned int sectorsPerBlock = file->blocksize / DISK_SECTORDATASIZE;

    // Calcula o primeiro setor dentro do bloco do inode para a leitura
		unsigned int firstSector = offset / DISK_SECTORDATASIZE;

    // Calcula o primeiro byte dentro do primeiro setor para a leitura
		unsigned int firstByteInSector = offset % DISK_SECTORDATASIZE;
    
    // Itera sobre os setores dentro de um bloco do inode
		for (int i = firstSector; i / DISK_SECTORDATASIZE < sectorsPerBlock && bytesRead < nbytes; i++)
		{
      // Lê um setor do disco para o buffer
			if (diskReadSector(file->disk, currentBlock + i, diskBuffer) == -1)
      {
        return -1; // Retorna -1 em caso de falha na leitura do setor
      }
				
      // Itera sobre os bytes dentro de um setor
			for (int j = firstByteInSector; j < DISK_SECTORDATASIZE && bytesRead < nbytes && bytesRead + file->lastByteRead < fileSize; j++)
			{
        // Copia os dados do buffer para o buffer de saída (buf)
				buf[bytesRead] = diskBuffer[j];
				bytesRead++;
			}

			firstByteInSector = 0; // Reinicia o índice do primeiro byte para o próximo setor
		}

		offset = 0; // Reinicia o offset para o próximo bloco do inode
		currentInodeBlockNumber++; // Avança para o próximo bloco do inode

    // Obtém o número do bloco no disco correspondente ao próximo bloco do inode
		currentBlock = inodeGetBlockAddr(file->inode, currentInodeBlockNumber); 
	}

  // Atualiza o último byte lido no arquivo
	file->lastByteRead += bytesRead;

	return bytesRead; // Retorna os bytes lidos
}

// Função para a escrita de um arquivo, a partir de um descritor de arquivo existente. 
// Os dados de buf serão copiados para o disco e terão tamanho máximo de nbytes. 
// Retorna o número de bytes efetivamente escritos em caso de sucesso ou -1, caso contrário.
int myFSWrite(int fd, const char *buf, unsigned int nbytes)
{
  // Verifica se o descritor de arquivo é válido
  if (fd <= 0 || fd > MAX_FDS) 
  {
      return -1;
  }
  
  // Obtém o ponteiro para o arquivo correspondente ao descritor de arquivo
  File *file = files[fd-1];

  // Verifica se o arquivo existe
  if (!file) 
  {
      return -1;
  }

  // Obtém o tamanho atual do arquivo
  unsigned int fileSize = inodeGetFileSize(file->inode);

  // Inicializa a variável para controlar o número de bytes efetivamente escritos
  unsigned int bytesWritten = 0;

  // Calcula o número do bloco do inode atual baseado no último byte lido
  unsigned int currentInodeBlockNumber = file->lastByteRead / file->blocksize;

  // Calcula o offset dentro do bloco do inode para a escrita
  unsigned int offset = file->lastByteRead % file->blocksize;

  // Obtém o número do bloco no disco correspondente ao bloco do inode atual
  unsigned int currentBlock = inodeGetBlockAddr(file->inode, currentInodeBlockNumber);

  // Inicializa um buffer para armazenar os dados lidos do disco
  unsigned char diskBuffer[DISK_SECTORDATASIZE];

  // Loop principal de escrita
  while (bytesWritten < nbytes)
  {
    // Calcula o número de setores por bloco do inode
    unsigned int sectorsPreBlock = file->blocksize / DISK_SECTORDATASIZE;

    // Calcula o primeiro setor dentro do bloco do inode para a escrita
    unsigned int firstSector = offset / DISK_SECTORDATASIZE;

    // Calcula o primeiro byte dentro do primeiro setor para a escrita
    unsigned int firstByteInSector = offset % DISK_SECTORDATASIZE;

    // Verifica se é necessário alocar um novo bloco no disco
    if (currentBlock == 0)
    {
      currentBlock = findFreeBlock(file->disk); // Encontra um bloco livre no disco

      // Verifica se foi possível encontrar um bloco livre
      if (currentBlock == -1)
      {
        break; // Sai do loop em caso de falha na alocação do bloco
      }
          
      // Adiciona o bloco ao inode do arquivo
      if (inodeAddBlock(file->inode, currentBlock) == -1)
      {
        // Libera o bloco no disco e sai do loop em caso de falha na adição do bloco ao inode
        setBlockFree(file->disk, currentBlock);
        break;
      }
    }

    // Itera sobre os setores dentro de um bloco do inode para a escrita
    for (int i = firstSector; i < sectorsPreBlock && bytesWritten < nbytes; i++)
    {
      // Lê um setor do disco para o buffer
      if (diskReadSector(file->disk, currentBlock + i, diskBuffer) == -1)
      {
        return -1; // Retorna -1 em caso de falha na leitura do setor
      }
          
      // Obtém o comprimento do caminho do arquivo
      int pathLength = strlen(file->path);

      // Itera sobre os bytes dentro de um setor para a escrita
      for (int j = firstByteInSector; j < DISK_SECTORDATASIZE && bytesWritten < nbytes; j++)
      {
        // Copia os dados do caminho do arquivo para o buffer do setor
        diskBuffer[j] = file->path[bytesWritten % pathLength];
        bytesWritten++;
      }

      // Escreve o buffer de volta ao disco
      if (diskWriteSector(file->disk, currentBlock + i, diskBuffer) == -1)
      {
        return -1; // Retorna -1 em caso de falha na escrita do setor
      }
          
      // Reinicia o índice do primeiro byte para o próximo setor
      firstByteInSector = 0;
    }

    offset = 0; // Reinicia o offset para o próximo bloco do inode
    currentInodeBlockNumber++; // Avança para o próximo bloco do inode

    // Obtém o número do bloco no disco correspondente ao próximo bloco do inode
    currentBlock = inodeGetBlockAddr(file->inode, currentInodeBlockNumber);
  }

  // Atualiza o tamanho do arquivo no inode
  inodeSetFileSize(file->inode, fileSize+bytesWritten);

  // Salva as alterações no inode no disco
  inodeSave(file->inode);

  return bytesWritten; // Retorna o número de bytes escritos
}

// Função para fechar um arquivo, a partir de um descritor de arquivo existente. 
// Retorna 0 caso bem sucedido, ou -1 caso contrário.
int myFSClose(int fd)
{
  // Verifica se o descritor de arquivo fornecido é válido
	if (fd <= 0 || fd > MAX_FDS)
  {
    return -1; // Retorna -1 se o descritor de arquivo não é válido
  }

  // Obtém o ponteiro para o arquivo correspondente ao descritor de arquivo
	File *file = files[fd-1];

  // Verifica se o arquivo existe
	if (!file)
  {
    return -1; // Retorna -1 se o arquivo não existe
  }
		
  // Libera a memória alocada para o inode associado ao arquivo
	free(file->inode);

  // Libera a memória alocada para a estrutura do arquivo
	free(file);

  // Define a posição correspondente ao descritor de arquivo como NULL no array de arquivos abertos
	files[fd-1] = NULL;

	return 0; // Retorna 0 indicando que o arquivo foi fechado com sucesso
}

// Função para instalar seu sistema de arquivos no S.O., registrando-o junto ao virtual FS (vfs). 
// Retorna um identificador único (slot), caso o sistema de arquivos tenha sido registrado com sucesso. Caso contrário, retorna -1
int installMyFS(void)
{
  fsInfo = malloc(sizeof(FSInfo));   // Alocação das informações do sistema
	fsInfo->fsname = "SistemaMatrix";   // Nome do Sistema
	fsInfo->fsid = (char)vfsRegisterFS(fsInfo);   // ID so sistema
	fsInfo->isidleFn = myFSIsIdle;   // Função para verificar se um arquivo está ocioso
	fsInfo->formatFn = myFSFormat;   // Função de formatação
	fsInfo->openFn = myFSOpen;   // Função para abrir/criar arquivo
	fsInfo->readFn = myFSRead;   // Função para leitura de bytes no arquivo
	fsInfo->writeFn = myFSWrite;   // Função para escrever bytes no arquivo
	fsInfo->closeFn = myFSClose;   // Função para fechar um arquivo
}