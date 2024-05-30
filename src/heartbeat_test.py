    def read_file(self, filename, source, hw_id):
        with open(filename, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                if int(row['hw_id']) == int(hw_id):
                    # print(f"Configuration for HW_ID {hw_id} found in {source}.")
                    return row
        return None