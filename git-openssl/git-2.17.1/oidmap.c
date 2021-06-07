#include "cache.h"
#include "oidmap.h"

static int cmpfn(const void *hashmap_cmp_fn_data,
		 const void *entry, const void *entry_or_key,
		 const void *keydata)
{
	const struct oidmap_entry *entry_ = entry;
	if (keydata)
		return oidcmp(&entry_->oid, (const struct object_id *) keydata);
	return oidcmp(&entry_->oid,
		      &((const struct oidmap_entry *) entry_or_key)->oid);
}

static int hash(const struct object_id *oid)
{
	int hash;
	memcpy(&hash, oid->hash, sizeof(hash));
	return hash;
}

void oidmap_init(struct oidmap *map, size_t initial_size)
{
	hashmap_init(&map->map, cmpfn, NULL, initial_size);
}

void oidmap_free(struct oidmap *map, int free_entries)
{
	if (!map)
		return;
	hashmap_free(&map->map, free_entries);
}

void *oidmap_get(const struct oidmap *map, const struct object_id *key)
{
	if (!map->map.cmpfn)
		return NULL;

	return hashmap_get_from_hash(&map->map, hash(key), key);
}

void *oidmap_remove(struct oidmap *map, const struct object_id *key)
{
	struct hashmap_entry entry;

	if (!map->map.cmpfn)
		oidmap_init(map, 0);

	hashmap_entry_init(&entry, hash(key));
	return hashmap_remove(&map->map, &entry, key);
}

void *oidmap_put(struct oidmap *map, void *entry)
{
	struct oidmap_entry *to_put = entry;

	if (!map->map.cmpfn)
		oidmap_init(map, 0);

	hashmap_entry_init(&to_put->internal_entry, hash(&to_put->oid));
	return hashmap_put(&map->map, to_put);
}
