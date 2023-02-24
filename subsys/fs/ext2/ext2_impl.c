/*
 * Copyright (c) 2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "ext2.h"
#include "ext2_impl.h"
#include "ext2_struct.h"
#include "ext2_diskops.h"

LOG_MODULE_REGISTER(ext2, CONFIG_EXT2_LOG_LEVEL);

static struct ext2_data __fs;
static bool initialized;

#define BLOCK_MEMORY_BUFFER_SIZE (CONFIG_EXT2_MAX_BLOCK_COUNT * CONFIG_EXT2_MAX_BLOCK_SIZE)
#define BLOCK_STRUCT_BUFFER_SIZE (CONFIG_EXT2_MAX_BLOCK_COUNT * sizeof(struct ext2_block))

/* Structures for blocks slab alocator */
struct k_mem_slab ext2_block_memory_slab, ext2_block_struct_slab;
char __aligned(sizeof(void *)) __ext2_block_memory_buffer[BLOCK_MEMORY_BUFFER_SIZE];
char __aligned(sizeof(void *)) __ext2_block_struct_buffer[BLOCK_STRUCT_BUFFER_SIZE];

/* Initialize heap memory allocator */
K_HEAP_DEFINE(ext2_heap, CONFIG_EXT2_HEAP_SIZE);

/* Helper functions --------------------------------------------------------- */

void *ext2_heap_alloc(size_t size)
{
	return k_heap_alloc(&ext2_heap, size, K_NO_WAIT);
}

void  ext2_heap_free(void *ptr)
{
	k_heap_free(&ext2_heap, ptr);
}

/* Block operations --------------------------------------------------------- */

static struct ext2_block *get_block_struct(void)
{
	int ret;
	struct ext2_block *b;

	ret = k_mem_slab_alloc(&ext2_block_struct_slab, (void **)&b, K_NO_WAIT);
	if (ret < 0) {
		LOG_ERR("get block: alloc block struct error %d", ret);
		return NULL;
	}

	ret = k_mem_slab_alloc(&ext2_block_memory_slab, (void **)&b->data, K_NO_WAIT);
	if (ret < 0) {
		LOG_ERR("get block: alloc block memory error %d", ret);
		k_mem_slab_free(&ext2_block_struct_slab, (void **)&b);
		return NULL;
	}
	return b;
}

struct ext2_block *ext2_get_block(struct ext2_data *fs, uint32_t block)
{
	int ret;
	struct ext2_block *b = get_block_struct();

	if (!b) {
		return NULL;
	}
	b->num = block;
	b->flags = EXT2_BLOCK_ASSIGNED;
	ret = fs->backend_ops->read_block(fs, b->data, block);
	if (ret < 0) {
		LOG_ERR("get block: read block error %d", ret);
		ext2_drop_block(fs, b);
		return NULL;
	}
	return b;
}

struct ext2_block *ext2_get_empty_block(struct ext2_data *fs)
{
	struct ext2_block *b = get_block_struct();

	if (!b) {
		return NULL;
	}
	b->num = 0;
	b->flags = 0;
	memset(b->data, 0, fs->block_size);
	return b;
}

int ext2_sync_block(struct ext2_data *fs, struct ext2_block *b)
{
	int ret;

	if (!(b->flags & EXT2_BLOCK_ASSIGNED)) {
		return -EINVAL;
	}

	ret = fs->backend_ops->write_block(fs, b->data, b->num);
	if (ret < 0) {
		return ret;
	}

	b->flags &= ~EXT2_BLOCK_DIRTY;
	return 0;
}

void ext2_drop_block(struct ext2_data *fs, struct ext2_block *b)
{
	if (b == NULL) {
		return;
	}

	if (b->flags & EXT2_BLOCK_DIRTY) {
		ext2_sync_block(fs, b);
	}

	if (b != NULL && b->data != NULL) {
		k_mem_slab_free(&ext2_block_memory_slab, (void **)&b->data);
		k_mem_slab_free(&ext2_block_struct_slab, (void **)&b);
	}
}

void ext2_init_blocks_slab(struct ext2_data *fs)
{
	memset(__ext2_block_memory_buffer, 0, BLOCK_MEMORY_BUFFER_SIZE);
	memset(__ext2_block_struct_buffer, 0, BLOCK_STRUCT_BUFFER_SIZE);

	/* These calls will always succeed because sizes and memory buffers are properly aligned. */

	k_mem_slab_init(&ext2_block_struct_slab, __ext2_block_struct_buffer,
			sizeof(struct ext2_block), CONFIG_EXT2_MAX_BLOCK_COUNT);

	k_mem_slab_init(&ext2_block_memory_slab, __ext2_block_memory_buffer, fs->block_size,
			CONFIG_EXT2_MAX_BLOCK_COUNT);
}

int ext2_assign_block_num(struct ext2_data *fs, struct ext2_block *b)
{
	int64_t new_block;

	if (b->flags & EXT2_BLOCK_ASSIGNED) {
		return -EINVAL;
	}

	/* Allocate block in the file system. */
	new_block = ext2_alloc_block(fs);
	if (new_block < 0) {
		return new_block;
	}

	b->num = new_block;
	b->flags |= EXT2_BLOCK_ASSIGNED;
	return 0;
}


/* FS operations ------------------------------------------------------------ */

int ext2_init_storage(struct ext2_data **fsp, const void *storage_dev, int flags)
{
	if (initialized) {
		return -EBUSY;
	}

	int ret = 0;
	struct ext2_data *fs = &__fs;
	int64_t dev_size, write_size;

	*fsp = fs;
	fs->open_inodes = 0;
	fs->flags = 0;

	ret = ext2_init_disk_access_backend(fs, storage_dev, flags);
	if (ret < 0) {
		return ret;
	}

	dev_size = fs->backend_ops->get_device_size(fs);
	if (dev_size < 0) {
		ret = dev_size;
		goto err;
	}

	write_size = fs->backend_ops->get_write_size(fs);
	if (write_size < 0) {
		ret = write_size;
		goto err;
	}

	if (write_size < 1024 && 1024 % write_size != 0) {
		ret = -EINVAL;
		LOG_ERR("expecting sector size that divides 1024 (got: %lld)", write_size);
		goto err;
	}

	LOG_DBG("Device size: %lld", dev_size);
	LOG_DBG("Write size: %lld", write_size);

	fs->device_size = dev_size;
	fs->write_size = write_size;

	initialized = true;
err:
	return ret;
}

int ext2_verify_superblock(struct ext2_disk_superblock *sb)
{
	/* Check if it is a valid Ext2 file system. */
	if (sb->s_magic != EXT2_MAGIC_NUMBER) {
		LOG_ERR("Wrong file system magic number (%x)", sb->s_magic);
		return -EINVAL;
	}

	/* For now we don't support file systems with frag size different from block size */
	if (sb->s_log_block_size != sb->s_log_frag_size) {
		LOG_ERR("Filesystem with frag_size != block_size is not supported");
		return -ENOTSUP;
	}

	/* Support only second revision */
	if (sb->s_rev_level != EXT2_DYNAMIC_REV) {
		LOG_ERR("Filesystem with revision %d is not supported", sb->s_rev_level);
		return -ENOTSUP;
	}

	if (sb->s_inode_size != EXT2_GOOD_OLD_INODE_SIZE) {
		LOG_ERR("Filesystem with inode size %d is not supported", sb->s_inode_size);
		return -ENOTSUP;
	}

	/* Check if file system may contain errors. */
	if (sb->s_state == EXT2_ERROR_FS) {
		LOG_WRN("File system may contain errors.");
		switch (sb->s_errors) {
		case EXT2_ERRORS_CONTINUE:
			break;

		case EXT2_ERRORS_RO:
			LOG_WRN("File system can be mounted read only");
			return -EROFS;

		case EXT2_ERRORS_PANIC:
			LOG_ERR("File system can't be mounted");
			/* panic or return that fs is invalid */
			__ASSERT(sb->s_state == EXT2_VALID_FS, "Error detected in superblock");
			return -EINVAL;
		default:
			LOG_WRN("Unknown option for superblock s_errors field.");
		}
	}

	if ((sb->s_feature_incompat & EXT2_FEATURE_INCOMPAT_FILETYPE) == 0) {
		LOG_ERR("File system without file type stored in de is not supported");
		return -ENOTSUP;
	}

	if ((sb->s_feature_incompat & ~EXT2_FEATURE_INCOMPAT_SUPPORTED) > 0) {
		LOG_ERR("File system can't be mounted. Incompat features %d not supported",
				(sb->s_feature_incompat & ~EXT2_FEATURE_INCOMPAT_SUPPORTED));
		return -ENOTSUP;
	}

	if ((sb->s_feature_ro_compat & ~EXT2_FEATURE_RO_COMPAT_SUPPORTED) > 0) {
		LOG_WRN("File system can be mounted read only. RO features %d detected.",
				(sb->s_feature_ro_compat & ~EXT2_FEATURE_RO_COMPAT_SUPPORTED));
		return -EROFS;
	}

	LOG_DBG("ino_cnt:%d blk_cnt:%d blk_per_grp:%d ino_per_grp:%d free_ino:%d free_blk:%d "
			"blk_size:%d ino_size:%d mntc:%d",
			sb->s_inodes_count, sb->s_blocks_count, sb->s_blocks_per_group,
			sb->s_inodes_per_group, sb->s_free_inodes_count, sb->s_free_blocks_count,
			1024 << sb->s_log_block_size, sb->s_inode_size, sb->s_mnt_count);
	return 0;
}

int ext2_init_fs(struct ext2_data *fs)
{
	int ret = 0;

	/* Fetch superblock */
	if (fs->block_size == 1024) {
		fs->sblock_offset = 0;
		fs->sblock = ext2_get_block(fs, 1);
	} else {
		fs->sblock_offset = 1024;
		fs->sblock = ext2_get_block(fs, 0);
	}

	if (fs->sblock == NULL) {
		ret = ENOENT;
		goto out;
	}

	if (!(fs->flags & EXT2_DATA_FLAGS_RO)) {
		/* Update sblock fields set during the successful mount. */
		EXT2_DATA_SBLOCK(fs)->s_state = EXT2_ERROR_FS;
		EXT2_DATA_SBLOCK(fs)->s_mnt_count += 1;
		fs->sblock->flags |= EXT2_BLOCK_DIRTY;
	}
	return 0;
out:
	ext2_drop_block(fs, fs->sblock);
	fs->sblock = NULL;
	return ret;
}

int ext2_close_fs(struct ext2_data *fs)
{
	int ret = 0;

	/* Close all open inodes */
	for (int32_t i = 0; i < fs->open_inodes; ++i) {
		if (fs->inode_pool[i] != NULL) {
			ext2_inode_drop(fs->inode_pool[i]);
		}
	}

	if (!(fs->flags & EXT2_DATA_FLAGS_RO)) {
		EXT2_DATA_SBLOCK(fs)->s_state = EXT2_VALID_FS;
		fs->sblock->flags |= EXT2_BLOCK_DIRTY;
	}

	/* free block group if it is fetched */
	if (fs->bgroup != NULL) {
		ext2_drop_block(fs, fs->bgroup->inode_table);
		ext2_drop_block(fs, fs->bgroup->inode_bitmap);
		ext2_drop_block(fs, fs->bgroup->block_bitmap);
		ext2_drop_block(fs, fs->bgroup->block);
	}
	ext2_drop_block(fs, fs->sblock);
	fs->sblock = NULL;
	return ret;
}

int ext2_close_struct(struct ext2_data *fs)
{
	memset(fs, 0, sizeof(struct ext2_data));
	initialized = false;
	return 0;
}

/* Lookup ------------------------------------------------------------------- */

/* Functions needed by lookup inode */
static const char *skip_slash(const char *str);
static char *strchrnul(const char *str, const char c);
static int64_t find_dir_entry(struct ext2_inode *inode, const char *name, size_t len,
		uint32_t *r_offset);

int ext2_lookup_inode(struct ext2_data *fs, struct ext2_lookup_args *args)
{
	LOG_DBG("Looking for file %s", args->path);

	int rc, ret = 0;
	struct ext2_inode *cur_dir = NULL, *next = NULL;
	static char name_buf[EXT2_MAX_FILE_NAME + 1];

	/* Start looking from root directory of file system */
	rc = ext2_inode_get(fs, EXT2_ROOT_INODE, &cur_dir);
	if (rc < 0) {
		ret = rc;
		goto out;
	}

	/* There may be slash at the beginning of path */
	const char *path = args->path;

	path = skip_slash(path);

	/* If path is empty then return root directory */
	if (path[0] == '\0') {
		args->inode = cur_dir;
		cur_dir = NULL;
		goto out;
	}

	for (;;) {
		/* Get path component */
		char *end = strchrnul(path, '/');
		size_t len = end - path;

		if (len > EXT2_MAX_FILE_NAME) {
			ret = -ENAMETOOLONG;
			goto out;
		}

		strncpy(name_buf, path, len);
		name_buf[len] = '\0';

		/* Search in current directory */
		uint32_t dir_off = 0;
		/* using 64 bit value to don't lose any information on error */
		int64_t ino = find_dir_entry(cur_dir, name_buf, len, &dir_off);

		const char *next_path = skip_slash(end);
		bool last_entry = next_path[0] == '\0';

		if (!last_entry) {
			/*  prepare the next loop iteration */

			if (ino < 0) {
				/* next entry not found */
				ret = -ENOENT;
				goto out;
			}

			rc = ext2_inode_get(fs, ino, &next);
			if (rc < 0) {
				/* error while fetching next entry */
				ret = rc;
				goto out;
			}

			if (!(next->i_mode & EXT2_S_IFDIR)) {
				/* path component should be directory */
				ret = -ENOTDIR;
				goto out;
			}

			/* Go to the next path component */
			path = next_path;

			/* Move to next directory */
			ext2_inode_drop(cur_dir);
			cur_dir = next;

			next = NULL;
			continue;
		}

		/* Last entry */

		if (ino < 0 && !(args->flags & LOOKUP_ARG_CREATE)) {
			/* entry not found but we need it */
			ret = -ENOENT;
			goto out;
		}

		if (ino > 0) {
			rc = ext2_inode_get(fs, ino, &next);
			if (rc < 0) {
				ret = rc;
				goto out;
			}
		}

		/* Store parent directory and offset in parent directory */
		if (args->flags & (LOOKUP_ARG_CREATE | LOOKUP_ARG_STAT | LOOKUP_ARG_UNLINK)) {
			/* In create it will be valid only if we have found existing file */
			args->offset = dir_off;
			args->parent = cur_dir;
			cur_dir = NULL;
		}

		/* Store name info */
		if (args->flags & LOOKUP_ARG_CREATE) {
			args->name_pos = path - args->path;
			args->name_len = len;
		}

		/* Store found inode */
		if (ino > 0) {
			args->inode = next;
			next = NULL;
		}
		goto out;
	}

out:
	/* Always free that inodes.
	 * If some of them is returned from function then proper pointer is set to NULL.
	 */
	ext2_inode_drop(cur_dir);
	ext2_inode_drop(next);
	return ret;
}

/* Return position of given char or end of string. */
static char *strchrnul(const char *s, char c)
{
	while ((*s != c) && (*s != '\0')) {
		s++;
	}
	return (char *) s;
}

static const char *skip_slash(const char *s)
{
	while ((*s == '/') && (*s != '\0')) {
		s++;
	}
	return s;
}

/**
 * @brief Find inode
 *
 * @note Inodes are 32 bit. When we return signed 64 bit number then we don't
 *       lose any information.
 *
 * @param r_offset If not NULL then offset in directory of that entry is written here.
 * @return Inode number or negative error code
 */
static int64_t find_dir_entry(struct ext2_inode *inode, const char *name, size_t len,
		uint32_t *r_offset)
{
	int rc;
	uint32_t block, block_off, offset = 0;
	struct ext2_data *fs = inode->i_fs;

	while (offset < inode->i_size) {
		block = offset / fs->block_size;
		block_off = offset % fs->block_size;

		rc = ext2_fetch_inode_block(inode, block);
		if (rc < 0) {
			return rc;
		}

		struct ext2_disk_dentry *de =
			(struct ext2_disk_dentry *)(inode_current_block_mem(inode) + block_off);

		if (len == de->de_name_len && strncmp(de->de_name, name, len) == 0) {

			if (r_offset) {
				/* Return offset*/
				*r_offset = offset;
			}

			return (int64_t)de->de_inode;
		}

		/* move to next directory entry */
		offset += de->de_rec_len;
	}

	return -EINVAL;
}

/* Create files and directories */

/* Allocate inode number and fill inode table with default values. */
static int ext2_create_inode(struct ext2_data *fs, struct ext2_inode *parent,
		struct ext2_inode *inode, int type)
{
	int32_t ino = ext2_alloc_inode(fs);

	if (ino < 0) {
		return ino;
	}

	/* Clear inode table entry */
	int rc = ext2_clear_inode(fs, ino);

	if (rc < 0) {
		return rc;
	}

	/* fill inode with correct data */
	inode->i_fs = fs;
	inode->flags = 0;
	inode->i_id = ino;
	inode->i_size = 0;
	inode->i_mode = type == FS_DIR_ENTRY_FILE ? EXT2_S_IFREG : EXT2_S_IFDIR;
	inode->i_links_count = 0;
	memset(inode->i_block, 0, 15 * 4);

	if (type == FS_DIR_ENTRY_DIR) {
		/* Block group current block is already fetched. We don't have to do it again.
		 * (It was done above in clear_inode function.)
		 */
		fs->bgroup->block->flags |= EXT2_BLOCK_DIRTY;
		current_disk_bgroup(fs->bgroup)->bg_used_dirs_count += 1;
	}

	rc = ext2_commit_inode(inode);
	return rc;
}

static int ext2_add_direntry(struct ext2_inode *dir, struct ext2_disk_dentry *entry)
{
	LOG_DBG("Adding entry: {in=%d type=%d name_len=%d} to directory (in=%d)",
			entry->de_inode, entry->de_file_type, entry->de_name_len, dir->i_id);

	int rc = 0;
	uint32_t block_size = dir->i_fs->block_size;
	uint32_t entry_size = sizeof(struct ext2_disk_dentry) + entry->de_name_len;

	if (entry_size > block_size) {
		return -EINVAL;
	}

	/* Find last entry */
	/* get last block and start from first entry on that block */
	int last_blk = (dir->i_size / block_size) - 1;

	rc = ext2_fetch_inode_block(dir, last_blk);
	if (rc < 0) {
		return rc;
	}

	uint32_t offset = 0;

	struct ext2_disk_dentry *de = 0;
	/* loop must be executed at least once, because block_size > 0 */
	while (offset < block_size) {
		de = (struct ext2_disk_dentry *)(inode_current_block_mem(dir) + offset);
		if (offset + de->de_rec_len == block_size) {
			break;
		}
		offset += de->de_rec_len;
	}


	uint32_t occupied = sizeof(struct ext2_disk_dentry) + de->de_name_len;

	/* Align to 4 bytes */
	occupied = ROUND_UP(occupied, 4);

	LOG_DBG("Occupied: %d total: %d needed: %d", occupied, de->de_rec_len, entry_size);

	if (de->de_rec_len - occupied >= entry_size) {
		/* Entry fits into current block */
		offset += occupied;
		de->de_rec_len = occupied;
		entry->de_rec_len = block_size - offset;

	} else {
		LOG_DBG("Allocating new block for directory");

		/* Have to allocate new block */
		rc = ext2_fetch_inode_block(dir, last_blk + 1);
		if (rc < 0) {
			return rc;
		}

		/* Increase size of directory */
		dir->i_size += block_size;
		rc = ext2_commit_inode(dir);
		if (rc < 0) {
			return rc;
		}
		rc = ext2_commit_inode_block(dir);
		if (rc < 0) {
			return rc;
		}

		/* New entry will start at offset 0 */
		offset = 0;
		entry->de_rec_len = block_size;
	}

	LOG_DBG("Writing entry {in=%d type=%d rec_len=%d name_len=%d} to block %d of inode %d",
			entry->de_inode, entry->de_file_type, entry->de_rec_len, entry->de_name_len,
			inode_current_block(dir)->num, dir->i_id);

	memcpy(inode_current_block_mem(dir) + offset, entry, entry_size);

	rc = ext2_commit_inode_block(dir);
	return rc;
}

int ext2_create_file(struct ext2_inode *parent, struct ext2_inode *new_inode,
		struct ext2_lookup_args *args)
{
	int rc, ret = 0;
	struct ext2_data *fs = parent->i_fs;

	rc = ext2_create_inode(fs, args->inode, new_inode, FS_DIR_ENTRY_FILE);
	if (rc < 0) {
		return rc;
	}

	struct ext2_disk_dentry *entry =
		ext2_heap_alloc(sizeof(struct ext2_disk_dentry) + args->name_len);

	if (entry == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	entry->de_inode = new_inode->i_id;
	entry->de_name_len = args->name_len;
	entry->de_file_type = EXT2_FT_REG_FILE;
	memcpy(entry->de_name, args->path + args->name_pos, entry->de_name_len);

	rc = ext2_add_direntry(parent, entry);
	if (rc < 0) {
		ret = rc;
		goto out;
	}

	/* Successfully added to directory */
	new_inode->i_links_count += 1;

	rc = ext2_commit_inode(new_inode);
	if (rc < 0) {
		ret = rc;
	}
out:
	ext2_heap_free(entry);
	return ret;
}

int ext2_create_dir(struct ext2_inode *parent, struct ext2_inode *new_inode,
		struct ext2_lookup_args *args)
{
	int rc, ret = 0;
	struct ext2_data *fs = parent->i_fs;
	uint32_t block_size = parent->i_fs->block_size;

	rc = ext2_create_inode(fs, args->inode, new_inode, FS_DIR_ENTRY_DIR);
	if (rc < 0) {
		return rc;
	}

	/* Directory must have at least one block */
	new_inode->i_size = block_size;

	struct ext2_disk_dentry *entry =
		ext2_heap_alloc(sizeof(struct ext2_disk_dentry) + args->name_len);

	if (entry == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	entry->de_inode = new_inode->i_id;
	entry->de_name_len = args->name_len;
	entry->de_file_type = EXT2_FT_DIR;
	memcpy(entry->de_name, args->path + args->name_pos, entry->de_name_len);

	rc = ext2_add_direntry(parent, entry);
	if (rc < 0) {
		ret = rc;
		goto out;
	}

	/* Successfully added to directory */
	new_inode->i_links_count += 1;

	/* Add "." directory entry */
	entry->de_inode = new_inode->i_id;
	entry->de_name_len = 1;
	entry->de_file_type = EXT2_FT_DIR;
	entry->de_rec_len = block_size;
	memcpy(entry->de_name, ".", 1);

	/* It has to be inserted manually */
	rc = ext2_fetch_inode_block(new_inode, 0);
	if (rc < 0) {
		ret = rc;
		goto out;
	}

	memcpy(inode_current_block_mem(new_inode), entry, sizeof(struct ext2_disk_dentry) + 1);

	new_inode->i_links_count += 1;

	/* Add ".." directory entry */
	entry->de_inode = parent->i_id;
	entry->de_name_len = 2;
	entry->de_file_type = EXT2_FT_DIR;
	memcpy(entry->de_name, "..", 2);

	rc = ext2_add_direntry(new_inode, entry);
	if (rc < 0) {
		ret = rc;
		goto out;
	}

	/* Successfully added to directory */
	parent->i_links_count += 1;

	rc = ext2_commit_inode_block(new_inode);
	if (rc < 0) {
		ret = rc;
	}

	rc = ext2_commit_inode_block(parent);
	if (rc < 0) {
		ret = rc;
	}

	/* Commit inodes after increasing link counts */
	rc = ext2_commit_inode(new_inode);
	if (rc < 0) {
		ret = rc;
	}

	rc = ext2_commit_inode(parent);
	if (rc < 0) {
		ret = rc;
	}
out:
	ext2_heap_free(entry);
	return ret;
}

int ext2_inode_get(struct ext2_data *fs, uint32_t ino, struct ext2_inode **ret)
{
	struct ext2_inode *inode;

	for (int i = 0; i < fs->open_inodes; ++i) {
		inode = fs->inode_pool[i];

		if (inode->i_id == ino) {
			*ret = inode;
			inode->i_ref++;
			return 0;
		}
	}

	if (fs->open_inodes >= MAX_INODES) {
		return -ENOMEM;
	}

	inode = ext2_heap_alloc(sizeof(struct ext2_inode));
	if (inode == NULL) {
		return -ENOMEM;
	}
	memset(inode, 0, sizeof(struct ext2_inode));

	if (ino != 0) {
		int rc = ext2_fetch_inode(fs, ino, inode);

		if (rc < 0) {
			ext2_heap_free(inode);
			return rc;
		}
	}

	fs->inode_pool[fs->open_inodes] = inode;
	fs->open_inodes++;

	inode->i_fs = fs;
	inode->i_ref = 1;
	*ret = inode;
	return 0;
}

int ext2_inode_drop(struct ext2_inode *inode)
{
	if (inode == NULL) {
		return 0;
	}

	struct ext2_data *fs = inode->i_fs;

	if (fs->open_inodes <= 0) {
		LOG_WRN("All inodes should be already closed");
		return 0;
	}

	inode->i_ref--;

	/* Clean inode if that was last reference */
	if (inode->i_ref == 0) {

		/* find entry */
		uint32_t offset = 0;

		while (fs->inode_pool[offset] != inode && offset < MAX_INODES) {
			offset++;
		}

		if (offset >= MAX_INODES) {
			LOG_ERR("Inode structure at %p not in inode_pool", inode);
			return -EINVAL;
		}

		ext2_inode_drop_blocks(inode);

		ext2_heap_free(inode);

		/* copy last open in place of freed inode */
		uint32_t last = fs->open_inodes - 1;

		fs->inode_pool[offset] = fs->inode_pool[last];
		fs->open_inodes--;

	}

	return 0;
}

void ext2_inode_drop_blocks(struct ext2_inode *inode)
{
	for (int i = 0; i < 4; ++i) {
		ext2_drop_block(inode->i_fs, inode->blocks[i]);
	}
	inode->flags &= ~INODE_FETCHED_BLOCK;
}
