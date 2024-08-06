'''
AI Chat Module Setup Assistant
Randy Mackay, December 2023

This script is used to setup the OpenAI assistant for the AI Chat Module.

OpenAI Assistant API: https://platform.openai.com/docs/api-reference/assistants
OpenAI Assistant Playground: https://platform.openai.com/playground

AP_FLAKE8_CLEAN
'''

import requests
import glob
import json
import xmltodict
import os

try:
    from openai import OpenAI
except Exception:
    print("chat: failed to import openai. See https://ardupilot.org/mavproxy/docs/modules/chat.html")
    exit()


# main function
def main(openai_api_key=None, assistant_name=None, model_name=None, upgrade=False):

    print("Starting assistant setup")

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
        print("setup_assistant: failed to connect to OpenAI.  Perhaps the API key was incorrect?")
        exit()

    # use assistant name if provided, otherwise use default
    if assistant_name is None:
        assistant_name = "ArduPilot Vehicle Control via MAVLink"

    # use model name if provided, otherwise use default
    if model_name is None:
        model_name = "gpt-4o"

    # check that assistant_instructions.txt file exists
    instructions_filename = os.path.join(os.getcwd(), "assistant_instructions.txt")
    if not os.path.isfile(instructions_filename):
        print("setup_assistant: " + instructions_filename + " not found")
        exit()

    # check that at least one text file exists.  E.g. text files holding the flight mode number to name mappings
    text_filenames = glob.glob(os.path.join(os.getcwd(), "*.txt"))
    if len(text_filenames) == 0:
        print("setup_assistant: no txt files found")
        exit()

    # check function definition files exist
    function_filenames = glob.glob(os.path.join(os.getcwd(), "*.json"))
    if len(function_filenames) == 0:
        print("setup_assistant: no function json files found")
        exit()

    # parse function definition files
    function_tools = [{"type": "file_search"}]
    for function_filename in function_filenames:
        try:
            # skip over files with xml in the name
            if "xml" in function_filename:
                print("skipping xml file: " + os.path.basename(function_filename))
                continue
            function_file = open(function_filename, 'r')
            function_object = json.load(function_file)
            function_tools.append(function_object)
            print("setup_assistant: parsed function file: " + os.path.basename(function_filename))
        except Exception:
            print("setup_assistant: failed to parse json file: " + function_filename)
            exit()

    # check that at least one function was parsed
    if len(function_tools) == 0:
        print("setup_assistant: no function json files found")
        exit()

    # download latest MAVLink files from ardupilot MAVLink repo, minimal.xml, common.xml and ardupilotmega.xml
    # convert to xml so they can be accessed by the assistant
    mavlink_filenames = ["minimal.xml", "common.xml", "ardupilotmega.xml"]
    mavlink_filenames_json = []
    for mavlink_filename in mavlink_filenames:
        if not download_file("https://raw.githubusercontent.com/ArduPilot/mavlink/master/message_definitions/v1.0/" + mavlink_filename, mavlink_filename):  # noqa
            exit()
        if not convert_xml_to_json(mavlink_filename):
            exit()
        mavlink_filenames_json.append(mavlink_filename + ".json")

    # variable to hold new assistant
    assistant = None

    # if assistant exists, use it
    assistants_list = client.beta.assistants.list()
    for existing_assistant in assistants_list.data:
        if existing_assistant.name == assistant_name:
            print("setup_assistant: using existing assistant: " + existing_assistant.name + " id:" + existing_assistant.id)
            assistant = existing_assistant
            break

    # exit if assistant was found but upgrade was not requested
    if assistant is not None and not upgrade:
        print("setup_assistant: assistant already exists, not upgrading")
        exit()

    # if assistant was not found, create it
    if assistant is None:
        assistant = client.beta.assistants.create(name=assistant_name, model=model_name, tools=function_tools)
        print("setup_assistant: created new assistant: " + assistant.name + " id:" + assistant.id)

    # update assistant's instructions
    try:
        instructions_content = open(instructions_filename, 'r').read()
        client.beta.assistants.update(assistant.id, instructions=instructions_content, tools=function_tools)
    except Exception:
        print("setup_assistant: failed to update assistant instructions")
        exit()

    # upload MAVLink and text files
    existing_files = client.files.list()
    uploaded_file_ids = []
    for filename in text_filenames + mavlink_filenames_json:
        try:
            # open local file as read-only
            file = open(filename, 'rb')
        except Exception:
            print("setup_assistant: failed to open file: " + filename)
            exit()

        # check if OpenAI has existing files with the same name
        filename_short = os.path.basename(filename)
        file_needs_uploading = True
        for existing_file in existing_files:
            if filename_short == existing_file.filename:
                # if not upgrading, we associate the existing file with our assistant
                if not upgrade:
                    uploaded_file_ids.append(existing_file.id)
                    print("setup_assistant: using existing file: " + existing_file.filename)
                    file_needs_uploading = False
                else:
                    # if upgrading them we delete and re-upload the file
                    # Note: this is slightly dangerous because files in use by another assistant could be deleted
                    try:
                        client.files.delete(existing_file.id)
                        print("setup_assistant: deleted existing file: " + existing_file.filename)
                    except Exception:
                        print("setup_assistant: failed to delete file from OpenAI: " + existing_file.filename)
                        exit()

        # upload file to OpenAI
        if file_needs_uploading:
            try:
                uploaded_file = client.files.create(file=file, purpose="assistants")
                uploaded_file_ids.append(uploaded_file.id)
                print("setup_assistant: uploaded: " + filename_short)
            except Exception:
                print("setup_assistant: failed to upload file to OpenAI: " + filename)
                exit()

    # delete and recreate a vector store on OpenAI
    vector_store_name = assistant_name + " vector_store"
    try:
        # retrieve list of vector stores
        vector_store_list = client.beta.vector_stores.list()
        for vs in vector_store_list:

            # check for valid names
            vs_name = vs.name
            if vs_name is None:
                vs_name = "no name"

            # delete empty vector stores
            if vs.file_counts.total == 0:
                print("deleting empty vector store id:" + vs.id + " name:" + vs_name)
                client.beta.vector_stores.delete(vector_store_id=vs.id)
                continue

            # delete our assistant's vector store if found
            if vs_name == vector_store_name:
                print("deleting existing vector store id:" + vs.id + " name:" + vs_name)
                client.beta.vector_stores.delete(vector_store_id=vs.id)
                break

        # create vector store for our assistant
        vector_store = client.beta.vector_stores.create(name=vector_store_name, file_ids=uploaded_file_ids)
        print("setup_assistant: created vector store id: " + vector_store.id +
              " name:" + vector_store_name + " files:" + str(vector_store.file_counts.total))

    except Exception as e:
        print("setup_assistant: failed to create vector store" + e)
        exit()

    # update assistant to use the vector store
    try:
        print("setup_assistant: updating assistant vector store to id:" + vector_store.id + " name:" + vector_store_name)
        client.beta.assistants.update(assistant.id, tool_resources={"file_search": {"vector_store_ids": [vector_store.id]}})
    except Exception:
        print("setup_assistant: failed to update assistant vector store")
        exit()

    # delete downloaded mavlink files
    for mavlink_filename in mavlink_filenames + mavlink_filenames_json:
        try:
            os.remove(mavlink_filename)
            print("setup_assistant: deleted local file: " + mavlink_filename)
        except Exception:
            print("setup_assistant: failed to delete file: " + mavlink_filename)

    # print completion message
    print("Assistant setup complete")


def download_file(url, filename):
    # download file from url to filename
    # return True if successful, False if not
    try:
        response = requests.get(url)
        if response.ok:
            file = open(os.path.join(os.getcwd(), filename), 'wb')
            file.write(response.content)
            file.close()
            print("setup_assistant: downloaded file: " + filename)
            return True
        else:
            print("setup_assistant: failed to download file: " + url)
            return False
    except Exception:
        print("setup_assistant: failed to download file: " + url)
        return False


def convert_xml_to_json(filename):
    # convert xml file to json file
    # return True if successful, False if not
    try:
        xml_file = open(filename)
        xml_file_dict = xmltodict.parse(xml_file.read())
        xml_file_json = json.dumps(xml_file_dict)
        json_file = open(filename + ".json", "w")
        json_file.write(xml_file_json)
        return True
    except Exception:
        print("setup_assistant: failed to convert xml to json: " + filename)
        return False


# call main function if this is run as standalone script
if __name__ == "__main__":

    # parse command line arguments
    from argparse import ArgumentParser
    parser = ArgumentParser(description="MAVProxy AI chat module OpenAI Assistant setup script")
    parser.add_argument("--api-key", default=None, help="OpenAI API Key")
    parser.add_argument("--name", default=None, help="Assistant name")
    parser.add_argument("--model", default=None, help="model name")
    parser.add_argument("--upgrade", action='store_true', help="upgrade existing assistant")
    args = parser.parse_args()
    main(openai_api_key=args.api_key, assistant_name=args.name, model_name=args.model, upgrade=args.upgrade)
