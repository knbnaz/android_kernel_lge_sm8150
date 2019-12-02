/*
 * Cryptographic API.
 *
 * Compression operations.
 *
 * Copyright (c) 2002 James Morris <jmorris@intercode.com.au>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 */
#include <linux/crypto.h>
#include "internal.h"

int crypto_comp_compress(struct crypto_comp *comp,
			 const u8 *src, unsigned int slen,
			 u8 *dst, unsigned int *dlen)
{
	struct crypto_tfm *tfm = crypto_comp_tfm(comp);

	return tfm->__crt_alg->cra_compress.coa_compress(tfm, src, slen, dst,
	                                                 dlen);
}
EXPORT_SYMBOL_GPL(crypto_comp_compress);

int crypto_comp_decompress(struct crypto_comp *comp,
			   const u8 *src, unsigned int slen,
			   u8 *dst, unsigned int *dlen)
{
	struct crypto_tfm *tfm = crypto_comp_tfm(comp);

	return tfm->__crt_alg->cra_compress.coa_decompress(tfm, src, slen, dst,
	                                                   dlen);
}
EXPORT_SYMBOL_GPL(crypto_comp_decompress);
