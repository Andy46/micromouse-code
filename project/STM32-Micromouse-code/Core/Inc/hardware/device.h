/*
 * device.h
 *
 *  Created on: Jul 26, 2024
 *      Author: agamb
 */

#pragma once

namespace HARDWARE
{

    /**
     * Device Interface class
     */
    class DEVICE
    {
    public:
        enum Status
        {
            NOT_INITIALIZED = 0,
            INITIALIZED,
            READY,
            ERROR,
        };

        Status getStatus() {return status;}
        bool isReady() {return status == Status::READY;}

    private:
        Status status;

    protected:
        DEVICE() : status(Status::NOT_INITIALIZED) {}
        virtual ~DEVICE() = default;

        void setInitialized() {status = Status::INITIALIZED;}
        void setReady() {status = Status::READY;}
        void setError() {status = Status::ERROR;}
    };

} /* namespace HARDWARE */
