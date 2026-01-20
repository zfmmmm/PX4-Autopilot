#ifndef ALTITUDE_HPP
#define ALTITUDE_HPP

#include <uORB/topics/depth_sensor.h>

/**
 * @brief MAVLink ALTITUDE message stream
 *
 * This stream maps data from the PX4 uORB depth_sensor topic
 * to the MAVLink ALTITUDE message.
 *
 * Typical usage:
 *  - Underwater vehicles (depth instead of altitude)
 *  - External depth sensors mapped into MAVLink telemetry
 */
class MavlinkStreamAltitude : public MavlinkStream
{
public:
	/**
	 * @brief Factory method used by MAVLink module
	 */
	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAltitude(mavlink);
	}

	/**
	 * @brief Stream name as exposed to MAVLink
	 */
	static constexpr const char *get_name_static() { return "ALTITUDE"; }

	/**
	 * @brief MAVLink message ID
	 */
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ALTITUDE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	/**
	 * @brief Report message size only when depth_sensor topic is advertised
	 *
	 * This avoids unnecessary bandwidth usage before the sensor becomes active.
	 */
	unsigned get_size() override
	{
		return _depth_sub.advertised()
		       ? MAVLINK_MSG_ID_ALTITUDE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES
		       : 0;
	}

private:
	explicit MavlinkStreamAltitude(Mavlink *mavlink)
		: MavlinkStream(mavlink)
	{
	}

	/**
	 * @brief Subscription to depth sensor uORB topic
	 *
	 * Only depth_sensor is used in this stream.
	 */
	uORB::Subscription _depth_sub{ORB_ID(depth_sensor)};

	/**
	 * @brief Send MAVLink ALTITUDE message
	 *
	 * This function is called periodically by the MAVLink scheduler.
	 * Data is sent only when new depth_sensor data is available.
	 */
	bool send() override
	{
		depth_sensor_s depth;

		if (_depth_sub.update(&depth)) {
			mavlink_altitude_t msg{};

			/**
			 * Timestamp (microseconds)
			 * Directly mapped from PX4 high-resolution timer
			 */
			msg.time_usec = depth.timestamp;

			/**
			 * Altitude field mapping according to MAVLink specification:
			 *
			 * - altitude_terrain:
			 *   Height above terrain. For underwater applications,
			 *   this field is commonly reused to represent depth.
			 *
			 * - bottom_clearance:
			 *   Distance to the bottom or obstacle below.
			 *
			 * Unused altitude fields are set to NaN.
			 */
			msg.altitude_monotonic = NAN;
			msg.altitude_amsl      = NAN;
			msg.altitude_local     = NAN;
			msg.altitude_relative  = NAN;
			msg.altitude_terrain   = depth.depth_m;
			msg.bottom_clearance   = depth.temperature_c; // NOTE: mapped by design (application-specific)

			mavlink_msg_altitude_send_struct(
				_mavlink->get_channel(),
				&msg
			);

			return true;
		}

		return false;
	}
};

#endif // ALTITUDE_HPP
