/**
 * Kamikaze QR location latitude
 *
 * @unit deg
 * @group NOVA_KMKZ
 */
PARAM_DEFINE_FLOAT(KMKZ_QR_LAT, 0.0f);

/**
 * Kamikaze QR location longitude
 *
 * @unit deg
 * @group NOVA_KMKZ
 */
PARAM_DEFINE_FLOAT(KMKZ_QR_LON, 0.0f);

/**
 * Kamikaze approach altitude
 *
 * @unit m
 * @min 20
 * @max 100
 * @group NOVA_KMKZ
 */
PARAM_DEFINE_INT32(KMKZ_APR_ALT, 80);

/**
 * Kamikaze approach latitude
 *
 * @unit deg
 * @group NOVA_KMKZ
 */
PARAM_DEFINE_FLOAT(KMKZ_APR_LAT, 0.0f);

/**
 * Kamikaze approach longitude
 *
 * @unit deg
 * @group NOVA_KMKZ
 */
PARAM_DEFINE_FLOAT(KMKZ_APR_LON, 0.0f);

/**
 * Kamikaze orbit radius
 *
 * @unit m
 * @min 15
 * @max 80
 * @group NOVA_KMKZ
 */
PARAM_DEFINE_INT32(KMKZ_ORB_RAD, 40);

/**
 * Kamikaze orbit side. 0 means left, 1 means right.
 *
 * @unit m
 * @min 0
 * @max 1
 * @group NOVA_KMKZ
 */
PARAM_DEFINE_INT32(KMKZ_ORB_SIDE, 1);

/**
 * Kamikaze dive start offset coefficient. Edit precisely
 *
 * @group NOVA_KMKZ
 */
PARAM_DEFINE_FLOAT(KMKZ_DIVE_OFS, 0.0f);

/**
 * Kamikaze approach start
 *
 * @boolean
 * @group NOVA_KMKZ
 */
PARAM_DEFINE_INT32(KMKZ_APR_START, 0);
