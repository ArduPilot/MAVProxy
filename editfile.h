
typedef void (*edit_file_callback_t)(const char *fname, int status);
void edit_file_background(const char *fname, edit_file_callback_t callback);
