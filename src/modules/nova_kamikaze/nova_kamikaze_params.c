/**
 * !! DO NOT CHANGE !! Nova Kamikaze module enabled
 *
 * @boolean
 * @group NOVA_KMKZ
 */
PARAM_DEFINE_INT32(NOVA_KMKZ, 1);

/**
 * Kamikaze dive angle
 *
 * @unit deg
 * @min 0
 * @max 60
 * @group NOVA_KMKZ
 */
PARAM_DEFINE_INT32(KMKZ_DIVE_ANG, 45);

/**
 * Kamikaze dive thrust
 *
 * @min 0
 * @max 1
 * @group NOVA_KMKZ
 */
PARAM_DEFINE_FLOAT(KMKZ_DIVE_THR, 0.2f);

/**
 * Kamikaze dive altitude
 *
 * @unit m
 * @min 20
 * @max 100
 * @group NOVA_KMKZ
 */
PARAM_DEFINE_INT32(KMKZ_DIVE_ALT, 30);


/**
 * Kamikaze level thrust
 *
 * @min 0
 * @max 1
 * @group NOVA_KMKZ
 */
PARAM_DEFINE_FLOAT(KMKZ_LEVEL_THR, 0.3f);

/**
 * Kamikaze rise angle
 *
 * @unit deg
 * @min 10
 * @max 30
 * @group NOVA_KMKZ
 */
PARAM_DEFINE_INT32(KMKZ_RISE_ANG, 15);

/**
 * Kamikaze rise thrust
 *
 * @min 0
 * @max 1
 * @group NOVA_KMKZ
 */
PARAM_DEFINE_FLOAT(KMKZ_RISE_THR, 0.8f);

/**
 * Kamikaze rise altitude
 *
 * @unit m
 * @min 40
 * @max 100
 * @group NOVA_KMKZ
 */
PARAM_DEFINE_INT32(KMKZ_RISE_ALT, 50);
