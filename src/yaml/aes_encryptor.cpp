#include "yaml/aes.h"
#include "yaml/aes_encryptor.h"
#include <cstring>
#include <cstdio>
#include <stdio.h>

using namespace std;

AesEncryptor::AesEncryptor(unsigned char* key)
{
  m_pEncryptor = new AES(key);
}

AesEncryptor::~AesEncryptor(void)
{
  delete m_pEncryptor;
}

void AesEncryptor::Byte2Hex(const unsigned char* src, int len, char* dest)
{
  for (int i = 0; i < len; ++i)
  {
    snprintf(dest + i * 2, 3, "%02X", src[i]);
  }
}

void AesEncryptor::Hex2Byte(const char* src, int len, unsigned char* dest)
{
  int length = len / 2;
  int temp;
  for (int i = 0; i < length; ++i)
  {
    temp = (Char2Int(src[i * 2]) * 16 + Char2Int(src[i * 2 + 1]));
    dest[i] = (unsigned char)temp;  //(unsigned char)(Char2Int(src[i * 2]) * 16 + Char2Int(src[i * 2 + 1]));
  }
}

int AesEncryptor::Char2Int(char c)
{
  if ((char)'0' <= c && c <= (char)'9')
  {
    return (int)(c - (char)'0');
  }
  else if ((char)'a' <= c && c <= (char)'f')
  {
    return (int)(c - (char)'a' + (char)10);
  }
  else if ((char)'A' <= c && c <= (char)'F')
  {
    return (int)(c - (char)'A' + (char)10);
  }
  return -1;
}

string AesEncryptor::EncryptString(string strInfor)
{
  int nLength = strInfor.length();
  int spaceLength = 16 - (nLength % 16);
  unsigned char* pBuffer = new unsigned char[nLength + spaceLength];
  memset(pBuffer, '\0', nLength + spaceLength);
  memcpy(pBuffer, strInfor.c_str(), nLength);
  m_pEncryptor->Cipher(pBuffer, nLength + spaceLength);

  // 这里需要把得到的字符数组转换成十六进制字符串
  char* pOut = new char[2 * (nLength + spaceLength)];
  memset(pOut, '\0', 2 * (nLength + spaceLength));
  Byte2Hex(pBuffer, nLength + spaceLength, pOut);

  string retValue(pOut);
  delete[] pBuffer;
  delete[] pOut;
  return retValue;
}

string AesEncryptor::DecryptString(string strMessage)
{
  int nLength = strMessage.length() / 2;
  unsigned char* pBuffer = new unsigned char[nLength];
  memset(pBuffer, '\0', nLength);
  Hex2Byte(strMessage.c_str(), strMessage.length(), pBuffer);

  m_pEncryptor->InvCipher(pBuffer, nLength);
  string retValue((char*)pBuffer);
  delete[] pBuffer;
  return retValue;
}
