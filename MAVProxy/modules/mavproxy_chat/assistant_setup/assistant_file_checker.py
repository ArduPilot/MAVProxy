'''
AI Chat Module Assistant File Checker
Randy Mackay, December 2023

This script is used to check for inconsistencies in the Assistants and Files.
E.g. files which are not used by any Assistants or Assistants which use files which do not exist.

OpenAI Assistant API: https://platform.openai.com/docs/api-reference/assistants
OpenAI Assistant Playground: https://platform.openai.com/playground

AP_FLAKE8_CLEAN
'''

import datetime
import os

try:
    from openai import OpenAI
except Exception:
    print("chat: failed to import openai. See https://ardupilot.org/mavproxy/docs/modules/chat.html")
    exit()


# main function
def main(openai_api_key=None, delete_unused=False):

    print("Starting Assistant File checker")

    # create connection object
    try:
        # if api key is provided, use it to create connection object
        if openai_api_key is not None:
            client = OpenAI(api_key=openai_api_key)
        else:
            # if no api key is provided, attempt to create connection object without it
            # user may have set the OPENAI_API_KEY environment variable
            client = OpenAI()
    except Exception:
        # if connection object creation fails, exit with error message
        print("assistant_file_checker: failed to connect to OpenAI.  Perhaps the API key was incorrect?")
        exit()

    # get a list of all Files
    existing_files = client.files.list()
    print("File list:")
    file_found = False
    for file in existing_files.data:
        print("    id:" + file.id + " name:" + file.filename + " size:" + str(file.bytes) + " date:" + str(datetime.datetime.fromtimestamp(file.created_at)))   # noqa
        file_found = True
    if not file_found:
        print("    no files found")

    # get a list of all assistants
    assistants_list = client.beta.assistants.list()

    # prepare a dictionary of file ids to the number of assistants using the file
    file_assistant_count = {}

    # can files be used by more than one assistant?

    # for each assistant, retrieve the list of files it uses
    for assistant in assistants_list.data:
        # print assistant name
        print("Assistant: " + assistant.name)

        # print list of files used by this assistant
        # increment the number of assistants associate with this file
        for assistant_file in client.beta.assistants.files.list(assistant_id=assistant.id):
            file_assistant_count[assistant_file.id] = file_assistant_count.get(assistant_file.id, 0) + 1
            filename_size_date = get_filename_size_date_from_id(existing_files, assistant_file.id)
            print("    id:" + assistant_file.id + " name:" + filename_size_date[0] + " size:" + str(filename_size_date[1]) + " date:" + filename_size_date[2])  # noqa

    # display the list of files which are not used by any Assistants
    print_unused_files_header = True
    for existing_file in existing_files:
        if existing_file.id not in file_assistant_count.keys():
            # print header if this is the first unused file
            if print_unused_files_header:
                print("Files not used by any Assistants:")
                print_unused_files_header = False

            # print file attributes
            filename_size_date = get_filename_size_date_from_id(existing_files, existing_file.id)
            print("    id:" + existing_file.id + " name:" + filename_size_date[0] + " size:" + str(filename_size_date[1]) + " date:" + filename_size_date[2])   # noqa

            # delete the unused file if specified by the user
            if delete_unused:
                client.files.delete(file_id=existing_file.id)
                print("        deleted: " + existing_file.id + " name:" + filename_size_date[0])

    # print completion message
    print("Assistant File check complete")


# searches the files_list for the specified file id and retuns the filename, size and creation date (as a string) if found
# returns None if not found
def get_filename_size_date_from_id(files_list, file_id):
    for file in files_list.data:
        if file.id == file_id:
            return file.filename, file.bytes, str(datetime.datetime.fromtimestamp(file.created_at))
    return "not found", "unknown", "unknown"


# call main function if this is run as standalone script
if __name__ == "__main__":

    # parse command line arguments
    from argparse import ArgumentParser
    parser = ArgumentParser(description="MAVProxy Chat Module Assistant File Checker")
    parser.add_argument("--api-key", default=None, help="OpenAI API Key")
    parser.add_argument("--delete-unused", action='store_true', help="delete unused files")
    try:
        args = parser.parse_args()
    except AttributeError:
        parser.print_help()
        os.sys.exit(0)
    main(openai_api_key=args.api_key, delete_unused=args.delete_unused)
