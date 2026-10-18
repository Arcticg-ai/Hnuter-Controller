"""Replace only the owned server launch; leave gz transport commands intact."""
def patch_startup(text,seed):
    if type(seed) is not int or seed not in (0,1,2,3,4):raise ValueError('registered seed must be 0, 1, 2, 3 or 4')
    old='${gz_command} ${gz_sub_command} --verbose=${GZ_VERBOSE:=1} -r -s "${PX4_GZ_WORLDS}/${PX4_GZ_WORLD}.sdf" &'
    if text.count(old)!=1:raise ValueError('unexpected upstream server command')
    new=f'/work/build/seeded_server {seed} "${{PX4_GZ_WORLDS}}/${{PX4_GZ_WORLD}}.sdf" "$JEPA_RUN_DIR/seed_receipt.json" &'
    return text.replace(old,new)
