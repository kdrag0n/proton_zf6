#
# linux/arch/arm64/crypto/Makefile
#
# Copyright (C) 2014 Linaro Ltd <ard.biesheuvel@linaro.org>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#

obj-$(CONFIG_CRYPTO_SHA1_ARM64_CE) += sha1-ce-glue.o sha1-ce-core.o

obj-$(CONFIG_CRYPTO_SHA2_ARM64_CE) += sha2-ce-glue.o sha2-ce-core.o

obj-$(CONFIG_CRYPTO_GHASH_ARM64_CE) += ghash-ce-glue.o ghash-ce-core.o

obj-$(CONFIG_CRYPTO_CRCT10DIF_ARM64_CE) += crct10dif-ce-core.o crct10dif-ce-glue.o

obj-$(CONFIG_CRYPTO_CRC32_ARM64_CE) += crc32-ce-core.o crc32-ce-glue.o

obj-$(CONFIG_CRYPTO_AES_ARM64_CE) += aes-ce-core.o aes-ce-glue.o

obj-$(CONFIG_CRYPTO_AES_ARM64_CE_CCM) += aes-ce-ccm-glue.o aes-ce-ccm-core.o

obj-$(CONFIG_CRYPTO_AES_ARM64_CE_BLK) += aes-glue-ce.o aes-ce.o

obj-$(CONFIG_CRYPTO_AES_ARM64_NEON_BLK) += aes-glue-neon.o aes-neon.o

obj-$(CONFIG_CRYPTO_SHA256_ARM64) += sha256-glue.o sha256-core.o

obj-$(CONFIG_CRYPTO_SHA512_ARM64) += sha512-glue.o sha512-core.o

obj-$(CONFIG_CRYPTO_CHACHA20_NEON) += chacha20-neon-core.o chacha20-neon-glue.o

obj-$(CONFIG_CRYPTO_AES_ARM64) += aes-cipher-core.o aes-cipher-glue.o

obj-$(CONFIG_CRYPTO_AES_ARM64_BS) += aes-neonbs-core.o aes-neonbs-glue.o

AFLAGS_aes-ce.o		:= -DINTERLEAVE=4
AFLAGS_aes-neon.o	:= -DINTERLEAVE=4

CFLAGS_aes-glue-ce.o	:= -DUSE_V8_CRYPTO_EXTENSIONS

$(obj)/aes-glue-%.o: $(src)/aes-glue.c FORCE
	$(call if_changed_rule,cc_o_c)

ifdef REGENERATE_ARM64_CRYPTO
quiet_cmd_perlasm = PERLASM $@
      cmd_perlasm = $(PERL) $(<) void $(@)

$(src)/sha256-core.S_shipped: $(src)/sha512-armv8.pl
	$(call cmd,perlasm)

$(src)/sha512-core.S_shipped: $(src)/sha512-armv8.pl
	$(call cmd,perlasm)
endif

.PRECIOUS: $(obj)/sha256-core.S $(obj)/sha512-core.S
