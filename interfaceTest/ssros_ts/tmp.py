# I need a function to get the file list from S3 bucket.
import boto3


def get_s3_file_list(bucket_name, path):
    """
    Gets a list of files from an S3 bucket within a specified path.

    Args:
        bucket_name (str): The name of the S3 bucket.
        path (str): The path within the bucket to list files from.

    Returns:
        list: A list of file names (keys) found in the specified path.
              Returns an empty list if no files are found or if the path doesn't exist.
    """
    s3 = boto3.client("s3")
    file_list = []

    # Ensure the path ends with a '/' if it's not empty, to treat it as a prefix
    if path and not path.endswith("/"):
        path += "/"

    paginator = s3.get_paginator("list_objects_v2")
    pages = paginator.paginate(Bucket=bucket_name, Prefix=path)

    for page in pages:
        if "Contents" in page:
            for obj in page["Contents"]:
                # Exclude the path itself if it's listed as an object (e.g., for empty folders)
                if obj["Key"] != path:
                    file_list.append(obj["Key"])

    return file_list
