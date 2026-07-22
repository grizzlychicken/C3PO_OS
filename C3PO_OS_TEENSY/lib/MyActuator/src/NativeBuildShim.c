
#ifndef PIO_UNIT_TESTING
#ifdef NATIVE_BUILD

// Satisfy linker in native environment without unit tests
int main(int argc, char ** argv)
{
    return 0;
}

#endif

#endif // !PIO_UNIT_TESTING