import yaml

def load_params(path, namespace):
    with open(path, 'r') as stream:
        try:
            params = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

        return params[namespace]['ros__parameters']